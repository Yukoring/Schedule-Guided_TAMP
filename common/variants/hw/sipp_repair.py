"""SIPP repair: one prioritized SIPP attempt for a colliding candidate (same assignment, order and actions)."""

import copy, re, time
from pathlib import Path
from types import SimpleNamespace as NS


def parse_for_sipp(task_prob, plan_file):
    """Plan text -> the prioritized solver's layout: per robot [loc1, loc2] for navigation (any *_navigate action) and
    [loc, duration, task, goal] for a service stop; tool stops as [loc, duration, 2, name] with an event record."""
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
    """One SIPP attempt for a colliding candidate. Returns the stored candidate record (valid or invalid) or None."""
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
                prm.task_roadmap,
                prm.task_samples,
                plan_list,
                events,
                rr=prm.max_rr,
                starts=starts,
                budget_check=ctx.budget.expired,
            )
        else:
            from motion.prioritized import PrioritizedPlanningSolver

            solver = PrioritizedPlanningSolver(
                prm.task_roadmap, prm.task_samples, plan_list, rr=prm.max_rr, starts=starts
            )
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
            sipp_rr=prm.max_rr,
            sipp_clearance=getattr(solver, "clearance", None),
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
            sipp_rr=prm.max_rr,
            sipp_clearance=getattr(solver, "clearance", None),
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
        sipp_rr=prm.max_rr,
        sipp_clearance=getattr(solver, "clearance", None),
        ok=rec["valid"],
        stage="validated" if rec["valid"] else "invalid",
        makespan=makespan,
        naive=meta.get("naive_pipeline_makespan"),
        violations=sorted({k for k, _ in rec["violations"]}) if not rec["valid"] else [],
        candidate_id=rec["candidate_id"],
    )
    return rec


def dispatch_to_sipp(dispatch):
    """Schedule-direct dispatch (per robot, time-sorted entries [pts, depart, travel] for moves and [[loc], start, dur, task, goal]
    for stops; tool stops carry task 2 and goal 'pickup_tool'/'drop_tool') -> the prioritized solver's layout plus tool events.
    """
    plan_list = {}
    events = []
    for robot, entries in dispatch.items():
        plan_list[robot] = []
        for e in sorted(entries, key=lambda x: x[1]):
            if len(e) == 3:
                pts = e[0]
                plan_list[robot].append([tuple(pts[0]), tuple(pts[-1])])
            else:
                loc = tuple(e[0][0])
                dur = e[2]
                task = e[3]
                goal = e[4]
                if task == 2:
                    plan_list[robot].append([loc, dur, 2, goal])
                    events.append(
                        {"t": e[1], "robot": robot, "kind": goal, "loc": loc, "robot_index": len(plan_list[robot]) - 1}
                    )
                else:
                    plan_list[robot].append([loc, dur, task, goal])
    # Fix , SD+SIPP tool v2): the hand-over adapter pairs each pickup with the preceding drop in EVENT ORDER, so the
    # events must be in global time order (the per-robot order produced above made every pickup location/holder check fail).
    events.sort(key=lambda e: (e["t"], e["robot"]))
    return plan_list, events


def _as_sample(loc, samples):
    """Dispatch locations may be lists or floats with rounding; map to the identical roadmap sample object."""
    if loc in samples:
        return loc
    best = min(samples, key=lambda s: (s[0] - loc[0]) ** 2 + (s[1] - loc[1]) ** 2)
    if (best[0] - loc[0]) ** 2 + (best[1] - loc[1]) ** 2 < 1e-6:
        return best
    raise ValueError("dispatch location %r is not a roadmap sample" % (loc,))


def repair_dispatch(ctx, vrp_prob, prm, dispatch, meta, count):
    """Schedule-direct + SIPP: one prioritized SIPP attempt on the scheduled task plan (order and assignment of the dispatch);
    returns the stored candidate record (valid or invalid) or None."""
    t0 = time.monotonic()
    env = ctx.world.env
    solver = None
    try:
        plan_list, events = dispatch_to_sipp(dispatch)
        samples = prm.task_samples
        for robot, legs in plan_list.items():
            for leg in legs:
                if len(leg) == 2:
                    leg[0] = _as_sample(leg[0], samples)
                    leg[1] = _as_sample(leg[1], samples)
                else:
                    leg[0] = _as_sample(leg[0], samples)
        for ev in events:
            ev["loc"] = _as_sample(ev["loc"], samples)
        starts = {n: r.pos for n, r in env.robots_map.items()}
        if ctx.tool:
            import baseline_tool_adapter as bta

            solver = bta.ToolPrioritizedSolver(
                prm.task_roadmap,
                samples,
                plan_list,
                events,
                rr=prm.max_rr,
                starts=starts,
                budget_check=ctx.budget.expired,
            )
        else:
            from motion.prioritized import PrioritizedPlanningSolver

            solver = PrioritizedPlanningSolver(prm.task_roadmap, samples, plan_list, rr=prm.max_rr, starts=starts)
        robot_path, makespan, total_cost = solver.find_solution()
    except Exception as e:
        ctx.add("motion", time.monotonic() - t0)
        ctx.event(type="sipp_repair", iteration=count, ok=False, stage="solver", error=repr(e)[:300])
        return None
    ctx.add("motion", time.monotonic() - t0)
    if not robot_path:
        ctx.event(
            type="sipp_repair",
            iteration=count,
            ok=False,
            stage="no_path",
            handover=getattr(solver, "handover_failure", None),
        )
        return None
    plan = NS(path=robot_path, motion_cost=makespan, total_cost=total_cost)
    collisions, _ = ctx.collide(robot_path, prm, [], None, count)
    ctx.motion_candidate(plan, {**meta, "repair_stage": "sipp"}, collisions)
    if collisions:
        ctx.event(
            type="sipp_repair",
            iteration=count,
            ok=False,
            stage="collisions_after_sipp",
            collisions=len(collisions),
            makespan=makespan,
        )
        return None
    rec = ctx.record(robot_path, copy.deepcopy(vrp_prob.obj_ins), makespan, meta, "sipp_repair")
    ctx.event(
        type="sipp_repair",
        iteration=count,
        ok=rec["valid"],
        stage="validated" if rec["valid"] else "invalid",
        makespan=makespan,
        violations=sorted({k for k, _ in rec["violations"]}) if not rec["valid"] else [],
        candidate_id=rec["candidate_id"],
    )
    return rec
