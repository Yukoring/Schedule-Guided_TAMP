#!/usr/bin/env python3
"""ITAGS adapter: exports the roadmap and tasks to the C++ search, runs it, and plans the resulting allocation with prioritized SIPP."""
import copy, hashlib, heapq, json, math, os, signal, subprocess, time
from pathlib import Path
from types import SimpleNamespace as NS

INF = float("inf")
RETREAT_CANDIDATES = 8  # candidate vertices for the final retreat
WAIT_ASIDE = True  # a precedence wait never happens within 2*clearance of the awaited service vertex
DIVERGENCE_ROUNDS = 6  # divergence detection of the release propagation


def effective_order(order, prec, who):
    """Precedence-aware priority: topological order of the robot dependencies, seeded by the rotation order."""
    edges = {
        (who[(g, a)], who[(g, b)]) for g, a, b in prec if (g, a) in who and (g, b) in who and who[(g, a)] != who[(g, b)]
    }
    rank = {n: i for i, n in enumerate(order)}
    out = []
    rem = list(order)
    while rem:
        avail = [n for n in rem if not any((q, n) in edges for q in rem)]
        pick = min(avail or rem, key=lambda n: rank[n])
        out.append(pick)
        rem.remove(pick)
    return out, sorted(edges)


def divergent(history):
    return len(history) >= DIVERGENCE_ROUNDS and len(set(history[-DIVERGENCE_ROUNDS:])) == 1


# ---- export
def dijkstra(adj, src, n):
    dist = [INF] * n
    dist[src] = 0.0
    h = [(0.0, src)]
    while h:
        d, u = heapq.heappop(h)
        if d > dist[u]:
            continue
        for v, w in adj[u]:
            if d + w < dist[v]:
                dist[v] = d + w
                heapq.heappush(h, (dist[v], v))
    return dist


def export_problem(world, out_path, budget_s, speed=1.0, alpha=0.5):
    """Write the ITAGS JSON for this run; returns the task mapping and the normalisation record."""
    env, prm = world.env, world.prm
    samples = [tuple(float(x) for x in p) for p in prm.task_samples]
    index = {p: i for i, p in enumerate(samples)}
    assert len(index) == len(samples), "duplicate task samples"
    vertices = [{"id": i, "x": float(p[0]), "y": float(p[1])} for i, p in enumerate(samples)]
    edges = []
    adj = [[] for _ in samples]
    seen = set()
    for i, nbrs in enumerate(prm.task_roadmap):
        for j in nbrs:
            j = int(j)
            if i == j:
                continue
            a, b = min(i, j), max(i, j)
            if (a, b) in seen:
                continue
            seen.add((a, b))
            w = math.dist(samples[a], samples[b])
            edges.append({"vertex_a": a, "vertex_b": b, "cost": w})
            adj[a].append((b, w))
            adj[b].append((a, w))
    robots = list(env.robots_map.items())
    goals = sorted(env.goals_map.items(), key=lambda kv: kv[1]["index"])
    services = sorted({s for _, g in goals for s in g["service"]})
    cost = {name: dict(zip(r.service, r.action_cost)) for name, r in robots}

    def cfg(i):
        return {
            "configuration_type": "graph",
            "graph_type": "point",
            "id": i,
            "x": vertices[i]["x"],
            "y": vertices[i]["y"],
        }

    species = []
    jrobots = []
    for name, r in robots:
        start = tuple(float(x) for x in r.pos)
        assert start in index, "robot start not a task sample: %s" % (start,)
        species.append(
            {
                "name": name + "_species",
                "traits": [1.0 if s in cost[name] else 0.0 for s in services],
                "bounding_radius": float(r.robot_radius),
                "speed": float(speed),
                "mp_index": 0,
            }
        )
        jrobots.append({"name": name, "species": name + "_species", "initial_configuration": cfg(index[start])})
    tasks = []
    mapping = []
    task_index = {}
    for gi, (gname, g) in enumerate(goals):
        loc = tuple(float(x) for x in world.goal_samples[g["index"]][0])
        assert loc in index, "goal sample not a task sample"
        for s in g["service"]:
            by_robot = {name: float(c[s]) for name, c in cost.items() if s in c}
            if not by_robot:
                raise ValueError("no eligible robot for %s/%s" % (gname, s))
            task_index[(gname, s)] = len(tasks)
            t = {
                "name": "%s_%s" % (gname, s),
                "duration": min(by_robot.values()),
                "desired_traits": [1.0 if x == s else 0.0 for x in services],
                "initial_configuration": cfg(index[loc]),
                "terminal_configuration": cfg(index[loc]),
            }
            if len(set(by_robot.values())) > 1:
                t["service_durations_by_robot"] = by_robot
            tasks.append(t)
            mapping.append(
                {
                    "task": len(tasks) - 1,
                    "goal": gname,
                    "service": s,
                    "vertex": index[loc],
                    "loc": list(loc),
                    "durations_by_robot": by_robot,
                }
            )
    precedence = []
    for gname, g in goals:
        for a, b in g.get("order", []):
            precedence.append([task_index[(gname, a)], task_index[(gname, b)]])
    # normalisation from the longest shortest path and the longest service duration
    key_vertices = sorted({index[tuple(float(x) for x in r.pos)] for _, r in robots} | {m["vertex"] for m in mapping})
    longest = 0.0
    for v in key_vertices:
        d = dijkstra(adj, v, len(samples))
        for u in key_vertices:
            if d[u] < INF:
                longest = max(longest, d[u])
    worst = sum(2 * longest / speed + max(m["durations_by_robot"].values()) for m in mapping)
    problem = {
        "alpha": alpha,
        "motion_planners": [
            {
                "environment_parameters": {
                    "configuration_type": "graph",
                    "graph_type": "point",
                    "vertices": vertices,
                    "edges": edges,
                },
                "mp_parameters": {"configuration_type": "graph", "timeout": 5.0},
                "mp_type": "point_graph_a_star",
            }
        ],
        "species": species,
        "robots": jrobots,
        "tasks": tasks,
        "precedence_constraints": precedence,
        "itags_parameters": {
            "has_timeout": True,
            "timeout": float(budget_s),
            "timer_name": "itags",
            "save_pruned_nodes": False,
            "save_closed_nodes": False,
        },
        "scheduler_parameters": {
            "scheduler_type": "cpsat",
            "time_scale": 1000,
            "relative_gap": 0.1,
            "timeout": 10.0,
            "threads": 4,
            "compute_transition_duration_heuristic": False,
            "use_hierarchical_objective": True,
        },
        "worst_makespan": worst,
        "plan_task_indices": list(range(len(tasks))),
    }
    Path(out_path).write_text(json.dumps(problem) + "\n")
    norm = {
        "longest_shortest_path_among_task_and_start_vertices": longest,
        "speed": speed,
        "worst_makespan": worst,
        "rule": "sum_tasks(2*longest/speed + max eligible duration); PRM-consistent replacement of the upstream perimeter rule",
        "vertices": len(vertices),
        "edges": len(edges),
        "tasks": len(tasks),
        "precedence": len(precedence),
        "services": services,
    }
    return mapping, norm, index


# ---- run the C++ search
def run_itags(binary, input_json, result_json, budget_s, cpus, log_path):
    argv = [
        "taskset",
        "-c",
        ",".join(map(str, cpus)),
        str(binary),
        str(input_json),
        str(result_json),
        "%.3f" % budget_s,
    ]
    t0 = time.monotonic()
    with open(log_path, "wb") as log:
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rc = None
    ru = None
    killed = False
    while True:
        pid, status, ru = os.wait4(proc.pid, os.WNOHANG)
        if pid:
            rc = os.waitstatus_to_exitcode(status)
            break
        if time.monotonic() - t0 > budget_s + 5.0:
            killed = True
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            pid, status, ru = os.wait4(proc.pid, 0)
            rc = os.waitstatus_to_exitcode(status)
            break
        time.sleep(0.02)
    proc.returncode = rc
    res = (
        json.loads(Path(result_json).read_text()) if Path(result_json).exists() else {"status": "exited_without_result"}
    )
    res["_process"] = {
        "argv": argv,
        "returncode": rc,
        "killed_by_adapter": killed,
        "wall_s": time.monotonic() - t0,
        "cpu_s": ru.ru_utime + ru.ru_stime,
        "utime_s": ru.ru_utime,
        "stime_s": ru.ru_stime,
        "maxrss_kb": ru.ru_maxrss,
    }
    return res


# ---- SIPP stage
def make_solver_class(PrioritizedPlanningSolver, Trajectory, _sipp, _SippCounter):
    class ItagsPrioritizedSolver(PrioritizedPlanningSolver):
        """Sequential SIPP with an explicit priority order and per-task release times (precedence propagation)."""

        def __init__(self, roadmap, samples, parsed_plan, order, releases, rr=0.3, starts=None):
            super().__init__(roadmap, samples, parsed_plan, rr, starts)
            self.order = list(order)
            self.releases = dict(releases)
            self.waits = []
            self.retreats = []

        def _apply(self, moves, breakpoints, t):
            for u, td, v, ta in moves:
                if td > t + 1e-9:
                    breakpoints.append((self.samples[u], td))
                breakpoints.append((self.samples[v], ta))
                t = ta
            return t

        def _release_leg(self, idx, goal_idx, t, rel, hold, trajectories, counter, breakpoints, name, task):
            """Reach the service vertex no earlier than the release and reserve the stay there; wait at the current vertex or aside."""
            aside = idx
            near = 2 * self.clearance
            if WAIT_ASIDE and math.dist(self.samples[idx], self.samples[goal_idx]) < near:
                cands = sorted(
                    (math.dist(self.samples[idx], sp), k)
                    for k, sp in enumerate(self.samples)
                    if math.dist(sp, self.samples[goal_idx]) >= near
                )
                for _, k in cands[:RETREAT_CANDIDATES]:
                    mv = _sipp(self.roadmap, self.samples, idx, k, t, 0.0, trajectories, self.clearance, counter)
                    if mv is not None:
                        aside = k
                        t = self._apply(mv, breakpoints, t)
                        break
                else:
                    print("no wait-aside vertex (agent %s)" % name)
                    return None
                self.waits.append(
                    {
                        "robot": name,
                        "task": task,
                        "wait_aside_at": list(self.samples[aside]),
                        "from": list(self.samples[idx]),
                        "until": rel,
                    }
                )
            probe = _sipp(self.roadmap, self.samples, aside, goal_idx, t, hold, trajectories, self.clearance, counter)
            if probe is None:
                print("no path to released service (agent %s)" % name)
                return None
            arrive = probe[-1][3] if probe else t
            if arrive < rel - 1e-9:
                depart = rel - (arrive - t)
                w = _sipp(
                    self.roadmap, self.samples, aside, aside, t, depart - t, trajectories, self.clearance, counter
                )
                if w is not None:
                    t = self._apply(w, breakpoints, t)
                    if t < depart:
                        breakpoints.append((self.samples[aside], depart))
                        t = depart
                    p2 = _sipp(
                        self.roadmap, self.samples, aside, goal_idx, t, hold, trajectories, self.clearance, counter
                    )
                    if p2 is not None:
                        probe = p2
                self.waits.append(
                    {
                        "robot": name,
                        "task": task,
                        "departure_delayed_to": depart,
                        "at": list(self.samples[aside]),
                        "until": rel,
                    }
                )
            t = self._apply(probe, breakpoints, t)
            if t < rel - 1e-9:
                breakpoints.append((self.samples[goal_idx], rel))
                t = rel
            return t, goal_idx

        def find_solution(self):
            import time as timer

            t0 = timer.time()
            trajectories = []
            counter = _SippCounter()
            pos = {n: i for i, n in enumerate(self.names)}
            seq = [pos[n] for n in self.order]
            future_goals = [[l[1] for l in legs if l[0] == "move"] for legs in self.legs]
            for k, i in enumerate(seq):
                name = self.names[i]
                if name in self.idle:
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
                for j, leg in enumerate(legs):
                    if leg[0] == "task":
                        rel = self.releases.get((leg[3], leg[2]), 0.0)
                        if rel > t + 1e-9:
                            # release while already at the service vertex: wait aside and come back
                            stay = 0.0
                            for l in legs[j:]:
                                if l[0] != "task":
                                    break
                                stay += l[1]
                            r_ = self._release_leg(
                                idx, idx, t, rel, stay, trajectories, counter, breakpoints, name, leg[2]
                            )
                            if r_ is None:
                                print("no precedence wait (agent %s leg %d)" % (name, j))
                                return None, -1, -1
                            t, idx = r_
                        events.append((t, leg[1], leg[2], leg[3]))
                        t += leg[1]
                        breakpoints.append((breakpoints[-1][0], t))
                        continue
                    goal_loc = leg[1]
                    goal_idx = self.samples.index(goal_loc)
                    hold = 0.0
                    rel = 0.0
                    if j + 1 < len(legs) and legs[j + 1][0] == "task":
                        # reserve the whole service stay at this vertex
                        hold = 0.0
                        for l in legs[j + 1 :]:
                            if l[0] != "task":
                                break
                            hold += l[1]
                        rel = self.releases.get((legs[j + 1][3], legs[j + 1][2]), 0.0)
                    if all(l[0] != "move" for l in legs[j + 1 :]):
                        hold = INF
                    if rel > 0:
                        # release on the way: delayed departure or wait-aside
                        r_ = self._release_leg(
                            idx, goal_idx, t, rel, hold, trajectories, counter, breakpoints, name, legs[j + 1][2]
                        )
                        if r_ is None:
                            print("no path (agent %s leg %d, %d states)" % (name, j, counter.expanded))
                            return None, -1, -1
                        t, idx = r_
                        continue
                    moves = _sipp(
                        self.roadmap, self.samples, idx, goal_idx, t, hold, trajectories, self.clearance, counter
                    )
                    if moves is None:
                        print("no path (agent %s leg %d, %d states)" % (name, j, counter.expanded))
                        return None, -1, -1
                    t = self._apply(moves, breakpoints, t)
                    idx = goal_idx
                mission_end = t if not events or events[-1][0] + events[-1][1] <= t else events[-1][0] + events[-1][1]
                retreat_dist = self.clearance * 2
                later = [g for ii in seq[k + 1 :] for g in future_goals[ii]]
                park = breakpoints[-1][0]
                if any(math.dist(park, g) < retreat_dist for g in later):
                    # retreat: nearest vertex clear of every later target; next-nearest tried when occupied
                    cands = sorted(
                        (math.dist(park, sp), kk)
                        for kk, sp in enumerate(self.samples)
                        if math.dist(park, sp) > 0 and all(math.dist(sp, g) >= retreat_dist for g in later)
                    )
                    moves = None
                    tried = 0
                    for _, target in cands[:RETREAT_CANDIDATES]:
                        tried += 1
                        moves = _sipp(
                            self.roadmap, self.samples, idx, target, t, INF, trajectories, self.clearance, counter
                        )
                        if moves is not None:
                            break
                    if cands and moves is None:
                        print("no retreat (agent %s, %d candidates)" % (name, tried))
                        return None, -1, -1
                    if moves is not None:
                        self.retreats.append(
                            {
                                "robot": name,
                                "from": list(park),
                                "to": list(self.samples[target]),
                                "candidate_rank": tried,
                            }
                        )
                        for u, td, v, ta in moves:
                            if td > t + 1e-9:
                                breakpoints.append((self.samples[u], td))
                            breakpoints.append((self.samples[v], ta))
                            t = ta
                trajectories.append(Trajectory(breakpoints))
                self._store(name, breakpoints, events)
                self.robot_paths[name]["mission_end"] = mission_end
                self.robot_paths[name]["breakpoints"] = breakpoints
            makespan = max(ele["mission_end"] for ele in self.robot_paths.values())
            total = sum(ele["mission_end"] for ele in self.robot_paths.values())
            if self.idle:
                horizon = max(
                    (ele["breakpoints"][-1][1] for n, ele in self.robot_paths.items() if n not in self.idle),
                    default=0.0,
                )
                for name in self.idle:
                    loc = self.starts[self.names.index(name)]
                    self.robot_paths[name]["path_cost"] = [[loc, horizon, [(horizon, 2)]]]
                    self.robot_paths[name]["path_only"] = [loc]
                    self.robot_paths[name]["path_time"] = horizon
            self.CPU_time = timer.time() - t0
            self.expanded = counter.expanded
            return self.robot_paths, makespan, total

    return ItagsPrioritizedSolver


def parsed_plan_from_itags(res, mapping, world, samples):
    """Per-robot legs in ITAGS service order."""
    names = list(world.env.robots_map)
    plan = {n: [] for n in names}
    orders = {}
    for rp in res["robot_plans"]:
        name = rp["name"]
        cur = tuple(world.env.robots_map[name].pos)
        legs = []
        order = []
        for t in rp["tasks"]:
            m = mapping[t]
            loc = tuple(m["loc"])
            dur = m["durations_by_robot"][name]
            if loc != cur:
                legs.append([cur, loc])
            legs.append([loc, dur, m["service"], m["goal"]])
            order.append((m["goal"], m["service"]))
            cur = loc
        plan[name] = legs
        orders[name] = order
    return plan, orders


def service_times(robot_path, timed_events):
    out = {}
    for r, p in robot_path.items():
        _, sv = timed_events(p["path_cost"])
        for t0, t1, task, job, pos in sv:
            out[(job, task)] = (t0, t1, r)
    return out


def rotations(names, k):
    n = len(names)
    return [names[i:] + names[:i] for i in range(min(k, n))]
