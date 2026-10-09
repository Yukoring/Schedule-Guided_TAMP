"""SIPP repair: one prioritized SIPP attempt for a colliding candidate (same assignment, order and actions)."""

import copy, re, time
from pathlib import Path
from types import SimpleNamespace as NS


def parse_for_sipp(task_prob, plan_file):
    """Plan text -> the prioritized solver layout: [loc1, loc2] for navigation, [loc, duration, task, goal] for a service stop, tool stops as [loc, duration, 2, name]."""
    text = Path(plan_file).read_text()
    ways = {w.ind: w.loc for w in task_prob.obj_ins["waypoint"]}
    rows = []
    for m in re.finditer(r"^\s*([0-9.]+):\s*\(([^)]+)\)\s*\[([0-9.]+)\]", text, re.M):
        t = float(m[1])
        parts = m[2].split()
        dur = float(m[3])
        rows.append((t, len(rows), parts, dur))
    plan_list = {r.ind: [] for r in task_prob.obj_ins["robot"]}
    events = []
    unknown = []
    for t, order, parts, dur in sorted(rows, key=lambda x: (x[0], x[1])):
        name = parts[0]
        robot = parts[1] if len(parts) > 1 else None
        if robot not in plan_list:
            unknown.append(name)
            continue
        if name.endswith("_navigate"):
            plan_list[robot].append([ways[parts[2]], ways[parts[3]]])
        elif name.startswith("do_task_single"):
            plan_list[robot].append([ways[parts[2]], dur, parts[4], parts[3]])
        elif name in ("pickup_tool", "drop_tool"):
            plan_list[robot].append([ways[parts[2]], dur, 2, name])
            events.append(
                {"t": t, "robot": robot, "kind": name, "loc": ways[parts[2]], "robot_index": len(plan_list[robot]) - 1}
            )
        else:
            unknown.append(name)
    if unknown:
        raise ValueError("unsupported plan actions for SIPP repair: %s" % sorted(set(unknown)))
    return plan_list, events


def repair(ctx, task_prob, prm, call, b, meta, count, col_env):
    """One SIPP attempt for a colliding candidate; returns the stored candidate record or None."""
    pf = call.dir / "candidate.pddl"
    if not pf.exists():
        return None
    t0 = time.monotonic()
    env = ctx.world.env
    solver = None
    try:
        plan_list, events = parse_for_sipp(task_prob, str(pf))
        starts = {n: r.pos for n, r in env.robots_map.items()}
        if ctx.tool:
            import baseline_tool_adapter as bta

            solver = bta.ToolPrioritizedSolver(
                prm.task_roadmap, prm.task_samples, plan_list, events, starts=starts, budget_check=ctx.budget.expired
            )
        else:
            from motion.prioritized import PrioritizedPlanningSolver

            solver = PrioritizedPlanningSolver(prm.task_roadmap, prm.task_samples, plan_list, starts=starts)
        robot_path, makespan, total_cost = solver.find_solution()
    except Exception as e:
        ctx.add("motion", time.monotonic() - t0)
        ctx.event(type="sipp_repair", call=call.id, ok=False, stage="solver", error=repr(e)[:300])
        return None
    ctx.add("motion", time.monotonic() - t0)
    if not robot_path:
        ctx.event(
            type="sipp_repair",
            call=call.id,
            ok=False,
            stage="no_path",
            handover=getattr(solver, "handover_failure", None),
            naive=meta.get("naive_pipeline_makespan"),
        )
        return None
    plan = NS(path=robot_path, motion_cost=makespan, total_cost=total_cost)
    collisions, _ = ctx.collide(robot_path, prm, [], None, count)
    ctx.motion_candidate(plan, {**meta, "repair_stage": "sipp"}, collisions)
    if collisions:
        ctx.event(
            type="sipp_repair",
            call=call.id,
            ok=False,
            stage="collisions_after_sipp",
            collisions=len(collisions),
            makespan=makespan,
            naive=meta.get("naive_pipeline_makespan"),
        )
        return None
    rec = ctx.record(robot_path, copy.deepcopy(task_prob.obj_ins), makespan, meta, "sipp_repair")
    ctx.event(
        type="sipp_repair",
        call=call.id,
        ok=rec["valid"],
        stage="validated" if rec["valid"] else "invalid",
        makespan=makespan,
        naive=meta.get("naive_pipeline_makespan"),
        violations=sorted({k for k, _ in rec["violations"]}) if not rec["valid"] else [],
        candidate_id=rec["candidate_id"],
    )
    return rec
