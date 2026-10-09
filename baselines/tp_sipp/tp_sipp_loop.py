#!/usr/bin/env python3
"""TP+SIPP baseline: one temporal plan refined by prioritized SIPP (shared motion layer), validated once; no feedback. Entry: prioritized_loop."""
import copy, hashlib, json, time
from pathlib import Path
from types import SimpleNamespace as NS
from control_common import *
from control_common import _single_problem_round


def prioritized_loop(ctx, gen_cls, domains, read_domain, solver_factory):
    env, prm = ctx.world.env, ctx.world.prm
    pool = ctx.pool
    d_name = read_domain(domains["early"])
    s0 = time.monotonic()
    makeprob = gen_cls(env, prm, d_name, "problem.pddl")
    makeprob.env_instance()
    ctx.add("setup", time.monotonic() - s0)
    g0 = time.monotonic()
    p_name = makeprob.generate_problem()
    ctx.add("pddl", time.monotonic() - g0)
    calls, rep, how = _single_problem_round(pool, domains["late"], p_name, 0)
    if rep is None:
        pool.terminate(calls, "killed_run_end")
        errs = [c for c in calls if error_exit(c)]
        return finish(
            ctx,
            "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
            "task plan: " + how,
            motion_attempts=0,
        )
    b = rep.candidate_block()
    pf = rep.dir / "candidate.pddl"
    pf.write_text(rep.plan_text(b))
    pool.terminate(calls, "killed_run_end")
    meta = {
        "call_id": rep.id,
        "source_label": label(rep.meta),
        "profile": rep.meta["profile"],
        "release": "late",
        "iteration": 0,
        "block": b["index"],
        "phase": how,
    }
    try:
        m0 = time.monotonic()
        parsed = makeprob.parse_plan(str(pf))
        solver = solver_factory(
            prm.task_roadmap, prm.task_samples, parsed, starts={n: r.pos for n, r in env.robots_map.items()}
        )
        robot_path, makespan, total_cost = solver.find_solution()
        ctx.add("motion", time.monotonic() - m0)
    except Exception as e:
        ctx.event(type="parser_error", call=rep.id, error=repr(e)[:300])
        return finish(ctx, "PARSER_ERROR", repr(e)[:160], motion_attempts=1)
    ctx.event(
        type="prioritized_motion",
        call=rep.id,
        source=label(rep.meta),
        success=bool(robot_path),
        makespan=makespan,
        handover_waits=getattr(solver, "handover_waits", None),
        idle_robots=sorted(getattr(solver, "idle", [])),
    )
    if not robot_path:
        return finish(
            ctx,
            "NO_PATH",
            "prioritized motion planning found no path; single attempt by protocol",
            motion_attempts=1,
            task_plan_source=label(rep.meta),
        )
    plan = NS(path=robot_path, motion_cost=makespan, total_cost=total_cost)
    collisions, _ = ctx.collide(robot_path, prm, [], None, 0)
    ctx.motion_candidate(plan, meta, collisions)
    if collisions:
        return finish(
            ctx,
            "MOTION_REJECTED",
            "shared collision checker: %d collision(s)" % len(collisions),
            motion_attempts=1,
            task_plan_source=label(rep.meta),
        )
    rec = ctx.record(robot_path, copy.deepcopy(makeprob.obj_ins), makespan, {**meta, "variant": 0}, "prioritized")
    if rec["valid"]:
        return finish(ctx, "VALID", candidate=rec, motion_attempts=1)
    return finish(
        ctx,
        "MOTION_REJECTED",
        "INVALID:" + "+".join(sorted({k for k, _ in rec["violations"]})),
        motion_attempts=1,
        task_plan_source=label(rep.meta),
    )
