#!/usr/bin/env python3
"""TP baseline: temporal planning only. Plans on the roadmap and resamples the roadmap after detected collisions; no schedule guidance, no SIPP.
Entry: planner_only_loop."""
import copy, hashlib, json, time
from pathlib import Path
from types import SimpleNamespace as NS
from control_common import *
from control_common import _single_problem_round


def planner_only_loop(ctx, gen_cls, domains, read_domain, prm_cls, log_prm):
    env = ctx.world.env
    prm = ctx.world.prm
    budget = ctx.budget
    pool = ctx.pool
    d_name = read_domain(domains["early"])
    n = 0
    iters = []
    while True:
        budget.check()
        s0 = time.monotonic()
        makeprob = gen_cls(env, prm, d_name, "problem.pddl")
        makeprob.env_instance()
        ctx.add("setup", time.monotonic() - s0)
        g0 = time.monotonic()
        p_name = makeprob.generate_problem(n)
        ctx.add("pddl", time.monotonic() - g0)
        calls, rep, how = _single_problem_round(pool, domains["late"], p_name, n)
        if rep is None:
            pool.terminate(calls, "killed_run_end")
            errs = [c for c in calls if error_exit(c)]
            return finish(
                ctx,
                "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
                "resample %d: %s" % (n, how),
                iterations=iters,
                prm_resamples=n,
            )
        plan, b = ctx.load(makeprob, rep, "Plan", 0)
        if plan is None:
            pool.terminate(calls, "killed_run_end")
            return finish(ctx, "PARSER_ERROR", "resample %d" % n, iterations=iters, prm_resamples=n)
        meta = {
            "call_id": rep.id,
            "source_label": label(rep.meta),
            "profile": rep.meta["profile"],
            "release": "late",
            "iteration": n,
            "block": b["index"],
            "phase": how,
        }
        next_col, col_env = ctx.collide(plan.path, prm, [], None, 0)
        ctx.motion_candidate(plan, meta, next_col)
        iters.append(
            {
                "iteration": n,
                "calls": [c.id for c in calls],
                "representative": rep.id,
                "source": label(rep.meta),
                "how": how,
                "colliding": bool(next_col),
            }
        )
        pool.terminate(calls, "killed_next_iteration" if next_col else "killed_run_end")
        if not next_col:
            rec = ctx.record(plan.path, makeprob.obj_ins, plan.motion_cost, {**meta, "variant": 0}, "final")
            if rec["valid"]:
                return finish(ctx, "VALID", candidate=rec, iterations=iters, prm_resamples=n)
            return finish(
                ctx,
                "NO_REPAIRABLE_CONFLICT",
                "INVALID:" + "+".join(sorted({k for k, _ in rec["violations"]})),
                iterations=iters,
                prm_resamples=n,
            )
        # resample the roadmap and plan again; no region feedback
        budget.check()
        r0 = time.monotonic()
        prm = prm_cls(env)
        init_samples, init_roadmap, goal_samples = prm.ConstructPhase(goals_map=env.goals_map)
        ctx.world.goal_samples = goal_samples
        n += 1
        ctx.add("prm_resample", time.monotonic() - r0)
        log_prm(n, init_samples, init_roadmap, goal_samples, prm)
