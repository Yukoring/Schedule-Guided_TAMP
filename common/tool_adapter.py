"""Portable tool extension only; original non-tool domains and trajectories unchanged."""

import copy, io, math, re
from pathlib import Path
from collections import defaultdict
from pddl_check import parse, emit, section, Replay
from task_planning.pddl_vrp_generator import Planner


def make_domain(source):
    tool = True
    root = parse(Path(source).read_text())
    if not tool:
        return emit(root) + "\n"
    section(root, ":predicates").extend(
        [
            ["tool_at", "?w", "-", "waypoint"],
            ["holding", "?r", "-", "robot"],
            ["tool_handler", "?r", "-", "robot"],
            ["needs_tool", "?s", "-", "jobtype"],
            ["tool_free_type", "?s", "-", "jobtype"],
        ]
    )
    extra_actions = []
    for a in root:
        if not isinstance(a, list) or not a or a[0] != ":durative-action":
            continue
        ci = a.index(":condition") + 1
        ei = a.index(":effect") + 1
        a[ci].append(["at", "start", ["robot_free", "?r"]])
        a[ei].extend([["at", "start", ["not", ["robot_free", "?r"]]], ["at", "end", ["robot_free", "?r"]]])
        if a[1] == "do_task_single":
            carrying = copy.deepcopy(a)
            carrying[1] = "do_task_single_tool"
            carrying[ci].extend([["at", "start", ["needs_tool", "?s"]], ["over", "all", ["holding", "?r"]]])
            extra_actions.append(carrying)
            a[ci].append(["at", "start", ["tool_free_type", "?s"]])
    root.extend(extra_actions)
    for name in ["pickup_tool", "drop_tool"]:
        pick = name == "pickup_tool"
        pre = ["tool_at", "?w"] if pick else ["holding", "?r"]
        post = ["holding", "?r"] if pick else ["tool_at", "?w"]
        root.append(
            [
                ":durative-action",
                name,
                ":parameters",
                ["?r", "-", "robot", "?w", "-", "waypoint"],
                ":duration",
                ["=", "?duration", "0.5"],
                ":condition",
                [
                    "and",
                    ["at", "start", ["robot_free", "?r"]],
                    ["at", "start", ["tool_handler", "?r"]],
                    ["over", "all", ["at", "?r", "?w"]],
                    ["at", "start", pre],
                ],
                ":effect",
                [
                    "and",
                    ["at", "start", ["not", ["robot_free", "?r"]]],
                    ["at", "start", ["not", pre]],
                    ["at", "end", post],
                    ["at", "end", ["robot_free", "?r"]],
                ],
            ]
        )
    return emit(root) + "\n"


class ToolPlanner(Planner):
    current = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tool = True
        ToolPlanner.current = self

    def generate_initial(self, info, const):
        original = self.pFile
        self.pFile = io.StringIO()
        super().generate_initial(info, const)
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


def partial_events(paths):
    result = []
    for r, p in paths.items():
        clock = 0.0
        prev = None
        for node in p["path_cost"]:
            loc = node[0] if isinstance(node, list) else node
            if prev is not None:
                clock += math.dist(prev, loc)
            if isinstance(node, list):
                for part in node[2]:
                    kind = part[2] if part[1] == 2 and len(part) > 2 else ("wait" if part[1] == 2 else "serve")
                    result.append(
                        {"robot": r, "start": clock, "end": clock + part[0], "kind": kind, "partial": part, "pos": loc}
                    )
                    clock += part[0]
            prev = loc
    return result


def validate_tools(planner, paths):
    if not planner.tool:
        return []
    ways = {w.ind: w.loc for w in planner.obj_ins["waypoint"]}
    g = next(g for g in planner.obj_ins["job"] if g.ind == "goal0000")
    available = ways[g.located[0]]
    holder = None
    busy = None
    events = defaultdict(list)
    errors = []
    active = set()
    for e in partial_events(paths):
        if e["kind"] in ["pickup_tool", "drop_tool"] or (
            e["kind"] == "serve" and e["partial"][1] in ["taska", "taskb"]
        ):
            events[round(e["start"], 7)].append(("start", e))
            events[round(e["end"], 7)].append(("end", e))
    for t, evs in sorted(events.items()):
        for r in active:
            if holder != r:
                errors.append("TOOL_NOT_HELD_DURING_SERVICE")
        for when, e in evs:
            r = e["robot"]
            kind = e["kind"]
            if kind == "serve":
                if when == "start":
                    if holder != r:
                        errors.append("TOOL_SERVICE_WITHOUT_POSSESSION")
                    active.add(r)
                else:
                    active.discard(r)
            elif when == "start":
                if busy is not None:
                    errors.append("TOOL_OPERATION_OVERLAP")
                if kind == "pickup_tool":
                    if available is None or math.dist(available, e["pos"]) > 1e-5:
                        errors.append("TOOL_PICKUP_LOCATION")
                    if holder is not None:
                        errors.append("TOOL_DOUBLE_HOLDER")
                    available = None
                else:
                    if holder != r:
                        errors.append("TOOL_DROP_WITHOUT_POSSESSION")
                    if active:
                        errors.append("TOOL_DROP_DURING_SERVICE")
                    holder = None
                busy = (r, kind)
                if abs(e["end"] - e["start"] - 0.5) > 1e-7:
                    errors.append("TOOL_ACTION_DURATION")
            else:
                if busy != (r, kind):
                    errors.append("TOOL_OPERATION_END")
                busy = None
                if kind == "pickup_tool":
                    holder = r
                else:
                    available = e["pos"]
    if busy is not None:
        errors.append("UNFINISHED_TOOL_ACTION")
    return sorted(set(errors))
