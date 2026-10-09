"""Portable tool for the temporal-planning baselines: tool domain, plan parsing and hand-over timing in prioritized SIPP."""

import copy, hashlib, io, math, re
from pathlib import Path
from pddl_check import parse, emit, section
import tool_adapter
from tool_adapter import validate_tools
from task_planning.pddlgenerator_vor import PDDLProblemGenerator
from motion.prioritized import PrioritizedPlanningSolver, Trajectory, _sipp, _SippCounter, INF

ADAPTER_VERSION = "baseline_portable_tool_v2_handover"
PRM_REGENERATIONS = [0]
HANDOVER_MAX_ROUNDS = 12  # cap on the hand-over propagation passes


def make_baseline_domain(source, workdir):
    root = parse(Path(source).read_text())
    preds = section(root, ":predicates")
    assert not any(isinstance(p, list) and p and p[0] == "robot_free" for p in preds[1:]), "robot_free already declared"
    preds.append(["robot_free", "?r", "-", "robot"])
    tmp = Path(workdir) / ("declared_" + Path(source).name)
    tmp.write_text(emit(root) + "\n")
    return tool_adapter.make_domain(tmp)


class ToolProblemGenerator(PDDLProblemGenerator):
    current = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tool = True
        self.tool_events = []
        ToolProblemGenerator.current = self

    def generate_initial(self):
        original = self.pFile
        self.pFile = io.StringIO()
        super().generate_initial()
        text = self.pFile.getvalue()
        self.pFile = original
        goal = next(g for g in self.obj_ins["job"] if g.ind == "goal0000")
        extra = [
            f"(tool_at {goal.located[0]})",
            "(needs_tool taska)",
            "(needs_tool taskb)",
            "(tool_free_type taskc)",
            "(tool_free_type taskd)",
        ]
        extra += [
            f"(tool_handler {r.ind})"
            for r in self.obj_ins["robot"]
            if any(t in r.action_duration for t in ["taska", "taskb"])
        ]
        extra += [f"(robot_free {r.ind})" for r in self.obj_ins["robot"]]
        text = text.rstrip()
        assert text.endswith(")")
        self.pFile.write(text[:-1] + "\n" + "\n".join(extra) + "\n)\n")

    def parse_pddl_plan(self, filename):
        dispatch, cost, elapsed = super().parse_pddl_plan(filename)
        ways = {w.ind: w.loc for w in self.obj_ins["waypoint"]}
        for m in re.finditer(
            r"^\s*([0-9.]+):\s*\((pickup_tool|drop_tool)\s+(\S+)\s+(\S+)\)\s*\[([0-9.]+)\]",
            Path(filename).read_text(),
            re.M,
        ):
            t, name, robot, way, d = m.groups()
            dispatch[robot].append([[ways[way]], float(t), float(d), 2, name])
        for actions in dispatch.values():
            actions.sort(key=lambda a: a[1])
        return dispatch, cost, elapsed

    def parse_plan(self, planFile):
        """Plan parsing with tool stops as 0.5 s stops; tool stops use task=2 and goal=pickup_tool/drop_tool."""
        text = Path(planFile).read_text()
        ways = {w.ind: w.loc for w in self.obj_ins["waypoint"]}
        rows = []
        for m in re.finditer(r"^\s*([0-9.]+):\s*\(([^)]+)\)\s*\[([0-9.]+)\]", text, re.M):
            t = float(m[1])
            parts = m[2].split()
            dur = float(m[3])
            rows.append((t, len(rows), parts, dur))
        plan_list = {r.ind: [] for r in self.obj_ins["robot"]}
        events = []
        for t, order, parts, dur in sorted(rows, key=lambda x: (x[0], x[1])):
            name = parts[0]
            robot = parts[1]
            if robot not in plan_list:
                continue
            if name.endswith("_navigate"):
                plan_list[robot].append([ways[parts[2]], ways[parts[3]]])
            elif name.startswith("do_task_single"):
                plan_list[robot].append([ways[parts[2]], dur, parts[4], parts[3]])
            elif name in ["pickup_tool", "drop_tool"]:
                plan_list[robot].append([ways[parts[2]], dur, 2, name])
                events.append(
                    {
                        "t": t,
                        "robot": robot,
                        "kind": name,
                        "loc": ways[parts[2]],
                        "robot_index": len(plan_list[robot]) - 1,
                    }
                )
        self.tool_events = events
        return plan_list


class ToolPrioritizedSolver(PrioritizedPlanningSolver):
    """Sequential SIPP with tool stops and hand-over timing: assignment, order and pickup/drop pairing are fixed; release times propagate from drop ends to pickups."""

    def __init__(self, roadmap, samples, parsed_plan, tool_events, rr=0.3, starts=None, budget_check=None):
        # budget_check: returns True when the run budget is exhausted
        # idle robots stay at their start
        super().__init__(roadmap, samples, parsed_plan, rr, starts)
        self.tool_events = tool_events
        self.budget_check = budget_check
        # predecessor drop of every pickup (None = initial tool position)
        self.pred = {}
        last_drop = None
        for k, e in enumerate(tool_events):
            if e["kind"] == "pickup_tool":
                self.pred[k] = last_drop
            else:
                last_drop = k
        self.handover_waits = []
        self.handover_rounds = 0
        self.handover_failure = None
        self.handover_history = []

    @staticmethod
    def _stop_run(legs, k):
        total = 0.0
        while k < len(legs) and legs[k][0] == "task":
            total += legs[k][1]
            k += 1
        return total

    def _apply(self, moves, breakpoints, t):
        for u, td, v, ta in moves:
            if td > t + 1e-9:
                breakpoints.append((self.samples[u], td))
            breakpoints.append((self.samples[v], ta))
            t = ta
        return t

    def _reach_not_before(self, idx, goal_idx, t, ready, hold, trajectories, counter, breakpoints, name, ev):
        """Reach the waypoint no earlier than `ready` and reserve the stay there; the receiver waits at its current vertex when that vertex is clear of every hand-over waypoint."""
        near = 2 * self.clearance
        aside = idx
        blocked = [self.samples[goal_idx]] + [tuple(e["loc"]) for e in self.tool_events if e["robot"] != name]
        if any(math.dist(self.samples[idx], b) < near for b in blocked):
            # the receiver may only wait where it is; too close to the hand-over waypoint -> failure
            print("receiver cannot wait in place for hand-over (agent %s)" % name)
            return None
        probe = _sipp(self.roadmap, self.samples, aside, goal_idx, t, hold, trajectories, self.clearance, counter)
        if probe is None:
            print("no path to hand-over waypoint (agent %s)" % name)
            return None
        arrive = probe[-1][3] if probe else t
        if arrive < ready - 1e-9:
            depart = ready - (arrive - t)
            w = _sipp(self.roadmap, self.samples, aside, aside, t, depart - t, trajectories, self.clearance, counter)
            if w is None:
                print("no hand-over wait (agent %s)" % name)
                return None
            t = self._apply(w, breakpoints, t)
            if t < depart:
                breakpoints.append((self.samples[aside], depart))
                t = depart
            p2 = _sipp(self.roadmap, self.samples, aside, goal_idx, t, hold, trajectories, self.clearance, counter)
            if p2 is None:
                print("no path to hand-over waypoint after wait (agent %s)" % name)
                return None
            probe = p2
            self.handover_waits.append(
                {
                    "robot": name,
                    "event": ev,
                    "departure_delayed_to": depart,
                    "at": list(self.samples[aside]),
                    "ready": ready,
                    "round": self.handover_rounds,
                }
            )
        t = self._apply(probe, breakpoints, t)
        if t < ready - 1e-9:
            breakpoints.append((self.samples[goal_idx], ready))
            t = ready
        return t, goal_idx

    def _plan_once(self, releases):
        """One sequential pass in the native priority order. Returns (ok, drop_end, pickup_start, fail_reason)."""
        trajectories = []
        counter = _SippCounter()
        drop_end = {}
        pickup_start = {}
        per_robot = {}
        for k, e in enumerate(self.tool_events):
            per_robot.setdefault(e["robot"], []).append(k)
        self.robot_paths = {n: dict() for n in self.names}
        for i, name in enumerate(self.names):
            if name in self.idle:
                # idle robot: stationary at its start
                breakpoints = [(self.starts[i], 0.0)]
                trajectories.append(Trajectory(breakpoints))
                self.robot_paths[name]["mission_end"] = 0.0
                self.robot_paths[name]["breakpoints"] = breakpoints
                continue
            idx = self.samples.index(self.starts[i])
            t = 0.0
            breakpoints = [(self.starts[i], 0.0)]
            events = []
            legs = self.legs[i]
            my_events = list(per_robot.get(name, []))

            def ready_of(ev):
                giver = self.pred.get(ev)
                if giver is None:
                    return 0.0  # first pickup: initial tool position, no predecessor drop
                r = releases.get(ev, 0.0)
                if giver in drop_end:
                    r = max(r, drop_end[giver])  # giver already planned in this pass: actual drop end
                return r

            j = 0
            while j < len(legs):
                leg = legs[j]
                if leg[0] == "task":
                    if leg[2] == 2 and leg[3] in ["pickup_tool", "drop_tool"]:
                        ev = my_events.pop(0)
                        if leg[3] == "pickup_tool":
                            ready = ready_of(ev)
                            if ready > t + 1e-9:
                                # already at the waypoint: wait and come back at ready
                                r_ = self._reach_not_before(
                                    idx,
                                    idx,
                                    t,
                                    ready,
                                    self._stop_run(legs, j),
                                    trajectories,
                                    counter,
                                    breakpoints,
                                    name,
                                    ev,
                                )
                                if r_ is None:
                                    return (
                                        False,
                                        drop_end,
                                        pickup_start,
                                        "hand-over wait failed (agent %s leg %d)" % (name, j),
                                    )
                                t, idx = r_
                            pickup_start[ev] = t
                        events.append((t, leg[1], leg[2], leg[3]))
                        t += leg[1]
                        breakpoints.append((breakpoints[-1][0], t))
                        if leg[3] == "drop_tool":
                            drop_end[ev] = t
                        j += 1
                        continue
                    events.append((t, leg[1], leg[2], leg[3]))
                    t += leg[1]
                    breakpoints.append((breakpoints[-1][0], t))
                    j += 1
                    continue
                goal_loc = leg[1]
                goal_idx = self.samples.index(goal_loc)
                # reserve the whole stop sequence (pickup, service, drop) after arrival
                hold = self._stop_run(legs, j + 1)
                if all(l[0] != "move" for l in legs[j + 1 :]):
                    # last navigation leg: parked at the final location
                    hold = INF
                ready = 0.0
                if (
                    j + 1 < len(legs)
                    and legs[j + 1][0] == "task"
                    and legs[j + 1][2] == 2
                    and legs[j + 1][3] == "pickup_tool"
                    and my_events
                ):
                    ready = ready_of(my_events[0])
                if ready > 0.0:
                    r_ = self._reach_not_before(
                        idx, goal_idx, t, ready, hold, trajectories, counter, breakpoints, name, my_events[0]
                    )
                    if r_ is None:
                        return (
                            False,
                            drop_end,
                            pickup_start,
                            "no path to hand-over (agent %s leg %d, %d states)" % (name, j, counter.expanded),
                        )
                    t, idx = r_
                    j += 1
                    continue
                moves = _sipp(self.roadmap, self.samples, idx, goal_idx, t, hold, trajectories, self.clearance, counter)
                if moves is None:
                    print("no path (agent %s leg %d, %d states)" % (name, j, counter.expanded))
                    return (
                        False,
                        drop_end,
                        pickup_start,
                        "no path (agent %s leg %d, %d states)" % (name, j, counter.expanded),
                    )
                t = self._apply(moves, breakpoints, t)
                idx = goal_idx
                j += 1
            mission_end = t if not events or events[-1][0] + events[-1][1] <= t else events[-1][0] + events[-1][1]
            trajectories.append(Trajectory(breakpoints))
            self._store(name, breakpoints, events)
            self.robot_paths[name]["mission_end"] = mission_end
            self.robot_paths[name]["breakpoints"] = breakpoints
        self.expanded = counter.expanded
        return True, drop_end, pickup_start, None

    def find_solution(self):
        import time as timer

        t0 = timer.time()
        releases = {}
        seen = set()
        self.handover_waits = []
        self.handover_history = []
        while True:
            if self.budget_check is not None and self.budget_check():
                # budget predicate; the worker reports this cause as BUDGET_EXHAUSTED
                self.handover_failure = {
                    "round": self.handover_rounds,
                    "cause": "budget exhausted during hand-over propagation",
                    "releases": {str(k): v for k, v in releases.items()},
                }
                self.CPU_time = timer.time() - t0
                return None, -1, -1
            self.handover_rounds += 1
            self.handover_waits = []
            ok, drop_end, pickup_start, why = self._plan_once(releases)
            if not ok:
                self.handover_failure = {
                    "round": self.handover_rounds,
                    "cause": why,
                    "releases": {str(k): v for k, v in releases.items()},
                }
                self.CPU_time = timer.time() - t0
                return None, -1, -1
            viol = [
                (ev, g, pickup_start[ev], drop_end[g])
                for ev, g in self.pred.items()
                if g is not None and ev in pickup_start and g in drop_end and pickup_start[ev] < drop_end[g] - 1e-9
            ]
            self.handover_history.append(
                {
                    "round": self.handover_rounds,
                    "releases": {str(k): v for k, v in releases.items()},
                    "pickups": {str(k): v for k, v in pickup_start.items()},
                    "drop_ends": {str(k): v for k, v in drop_end.items()},
                    "violations": [[ev, g, ps, de] for ev, g, ps, de in viol],
                }
            )
            if not viol:
                break
            # release = drop end + vacate margin
            margin = 2 * self.clearance + 0.2
            for ev, g, ps, de in viol:
                releases[ev] = max(releases.get(ev, 0.0), de + margin)
            key = tuple(sorted((k, round(v, 6)) for k, v in releases.items()))
            if key in seen:
                self.handover_failure = {
                    "round": self.handover_rounds,
                    "cause": "hand-over propagation did not converge: repeated release state",
                    "releases": {str(k): v for k, v in releases.items()},
                    "violations": [[ev, g, ps, de] for ev, g, ps, de in viol],
                }
                self.CPU_time = timer.time() - t0
                return None, -1, -1
            if self.handover_rounds >= HANDOVER_MAX_ROUNDS:
                self.handover_failure = {
                    "round": self.handover_rounds,
                    "cause": "hand-over propagation did not converge within %d rounds" % HANDOVER_MAX_ROUNDS,
                    "releases": {str(k): v for k, v in releases.items()},
                    "violations": [[ev, g, ps, de] for ev, g, ps, de in viol],
                }
                self.CPU_time = timer.time() - t0
                return None, -1, -1
            seen.add(key)
        makespan = max(ele["mission_end"] for ele in self.robot_paths.values())
        total_cost = sum(ele["mission_end"] for ele in self.robot_paths.values())
        if self.idle:
            # idle robot: stationary for the whole horizon
            horizon = max(
                (ele["breakpoints"][-1][1] for n, ele in self.robot_paths.items() if n not in self.idle), default=0.0
            )
            for name in self.idle:
                loc = self.starts[self.names.index(name)]
                self.robot_paths[name]["path_cost"] = [[loc, horizon, [(horizon, 2)]]]
                self.robot_paths[name]["path_only"] = [loc]
                self.robot_paths[name]["path_time"] = horizon
        self.CPU_time = timer.time() - t0
        print(
            "Prioritized SIPP (tool, v2 hand-over): %d states expanded in the final pass, %d round(s), %.2fs"
            % (self.expanded, self.handover_rounds, self.CPU_time)
        )
        return self.robot_paths, makespan, total_cost


def _tool_solver_factory(roadmap, samples, parsed_plan, rr=0.3, starts=None, budget_check=None):
    gen = ToolProblemGenerator.current
    return ToolPrioritizedSolver(roadmap, samples, parsed_plan, gen.tool_events, rr, starts, budget_check)


def wrapped_validator(native):
    def validate_final(paths, objects, robots):
        return native(paths, objects, robots) + [
            (e, "portable tool") for e in validate_tools(ToolProblemGenerator.current, paths)
        ]

    return validate_final


def install(method_mod, workdir):
    """Patch one native pipeline module for the tool families. Returns metadata for the run record."""
    workdir = Path(workdir)
    domains = {}
    for name in ["tp_domain_early.pddl", "tp_domain_late.pddl"]:
        text = make_baseline_domain(Path(__file__).resolve().parent / "code/pddl_domain" / name, workdir)
        (workdir / name).write_text(text)
    early = str(workdir / "tp_domain_early.pddl")
    late = str(workdir / "tp_domain_late.pddl")
    method_mod.DOMAIN_NAME_FILE = early
    if hasattr(method_mod, "DOMAIN_EARLY"):
        method_mod.DOMAIN_EARLY = early
    method_mod.DOMAIN_LATE = late
    method_mod.PDDLProblemGenerator = ToolProblemGenerator
    method_mod.validate_final = wrapped_validator(method_mod.validate_final)
    if hasattr(method_mod, "PrioritizedPlanningSolver"):
        method_mod.PrioritizedPlanningSolver = _tool_solver_factory
    if hasattr(method_mod, "PRMPlanning"):
        native_prm = method_mod.PRMPlanning

        class CountingPRM(native_prm):
            def __init__(self, *a, **k):
                PRM_REGENERATIONS[0] += 1
                super().__init__(*a, **k)

        method_mod.PRMPlanning = CountingPRM
    return {
        "adapter": ADAPTER_VERSION,
        "domains": domains,
        "method": method_mod.__name__,
    }
