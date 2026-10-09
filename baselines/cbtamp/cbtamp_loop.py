#!/usr/bin/env python3
"""CBTAMP baseline (Lee and Long 2023): temporal planning on the roadmap; collision regions found in candidate trajectories revise the temporal
planning problem; repeated until a validated plan or the budget ends. Shares roadmap, planner, validator with the other methods. Entry: cbtamp_loop."""
import copy, hashlib, json, time
from pathlib import Path
from types import SimpleNamespace as NS
from control_common import *
from control_common import _single_problem_round


def cbtamp_loop(ctx, gen_cls, domains, read_domain):
    """CBTAMP: an initial planning round, then repair rounds in which collision regions revise the planning problem; candidates are adopted by the original CBTAMP rule."""
    env, prm = ctx.world.env, ctx.world.prm
    budget = ctx.budget
    pool = ctx.pool
    d_name = read_domain(domains["early"])
    s0 = time.monotonic()
    makeprob = gen_cls(env, prm, d_name, "problem.pddl")
    makeprob.env_instance()
    ctx.add("setup", time.monotonic() - s0)
    g0 = time.monotonic()
    p_name = makeprob.generate_problem()
    ctx.add("pddl", time.monotonic() - g0)
    count = 0
    col_env = []
    iters = []

    def produce(count):
        mkA = lambda: [
            pool.new_call(
                domains[rel], p_name, flags=CS, profile="cS", release=rel, phase="A", geometry_iteration=count
            )
            for rel in ("early", "late")
        ]
        mkB = lambda: [
            pool.new_call(domains[rel], p_name, flags=N, profile="n", release=rel, phase="B", geometry_iteration=count)
            for rel in ("early", "late")
        ]
        A, B = two_phase(pool, mkA, mkB)
        calls = A + B
        ready = [c for c in calls if c.has_block()]
        how = "base"
        if not ready:
            arrivals, reason = pool.extend(calls)
            ready = arrivals
            how = "extension" if reason == "candidate" else reason
        return calls, ready, how

    def meta_of(c, b, how):
        return {
            "call_id": c.id,
            "source_label": label(c.meta),
            "profile": c.meta["profile"],
            "release": c.meta["release"],
            "iteration": c.meta["geometry_iteration"],
            "block": b["index"],
            "phase": how,
        }

    calls, ready, how = produce(0)
    if not ready:
        errs = [c for c in calls if error_exit(c)]
        pool.terminate(calls, "killed_run_end")
        return finish(
            ctx,
            "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
            "initial round: " + how,
            iterations=iters,
        )
    chosen = representative(ready)
    init, b = ctx.load(makeprob, chosen, "Initial Plan", 0)
    if init is None:
        pool.terminate(calls, "killed_run_end")
        return finish(ctx, "PARSER_ERROR", "initial plan", iterations=iters)
    early_col, col_env = ctx.collide(init.path, prm, col_env, None, 0)
    ctx.motion_candidate(init, meta_of(chosen, b, how), early_col)
    iters.append(
        {
            "iteration": 0,
            "calls": [c.id for c in calls],
            "ready": [c.id for c in ready],
            "chosen": chosen.id,
            "chosen_source": label(chosen.meta),
            "how": how,
            "colliding": bool(early_col),
        }
    )
    if not early_col:
        rec = ctx.record(
            init.path,
            makeprob.obj_ins,
            init.motion_cost,
            {**meta_of(chosen, b, how), "variant": 0 if chosen.meta["release"] == "early" else 1},
            "initial",
        )
        pool.terminate(calls, "killed_run_end")
        if rec["valid"]:
            return finish(ctx, "VALID", candidate=rec, iterations=iters)
        return finish(
            ctx,
            "NO_REPAIRABLE_CONFLICT",
            "INVALID:" + "+".join(sorted({k for k, _ in rec["violations"]})),
            iterations=iters,
        )
    next_collisions = early_col
    while next_collisions:
        pool.terminate(calls, "killed_next_iteration")
        budget.check()
        count += 1
        s0 = time.monotonic()
        makeprob.env_update(col_env)
        ctx.add("setup", time.monotonic() - s0)
        g0 = time.monotonic()
        p_name = makeprob.generate_problem(count)
        ctx.add("pddl", time.monotonic() - g0)
        calls, ready, how = produce(count)
        if not ready:
            errs = [c for c in calls if error_exit(c)]
            pool.terminate(calls, "killed_run_end")
            return finish(
                ctx,
                "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
                "iteration %d: %s" % (count, how),
                iterations=iters,
            )
        rep_e = representative([c for c in ready if c.meta["release"] == "early"])
        rep_l = representative([c for c in ready if c.meta["release"] == "late"])
        early, eb = ctx.load(makeprob, rep_e, "Early Plan", count) if rep_e else (None, None)
        late, lb = ctx.load(makeprob, rep_l, "Late Plan", count) if rep_l else (None, None)
        if not early and not late:
            pool.terminate(calls, "killed_run_end")
            return finish(ctx, "PARSER_ERROR", "iteration %d" % count, iterations=iters)
        early_col, late_col = [], []
        env_e = env_l = None
        if early:
            early_col, env_e = ctx.collide(early.path, prm, col_env, None, 0)
            ctx.motion_candidate(early, meta_of(rep_e, eb, how), early_col)
        if late:
            base = -1 if early_col else col_env
            late_col, env_l = ctx.collide(late.path, prm, base, None, 0)
            ctx.motion_candidate(late, meta_of(rep_l, lb, how), late_col)
        next_collisions = []
        valid = []
        invalid = []
        if late:
            if not late_col:
                rec = ctx.record(
                    late.path,
                    makeprob.obj_ins,
                    late.motion_cost,
                    {**meta_of(rep_l, lb, how), "variant": 1},
                    "round_candidate",
                )
                (valid if rec["valid"] else invalid).append(rec)
            else:
                next_collisions = late_col
                col_env = env_l
        if early:
            if not early_col:
                rec = ctx.record(
                    early.path,
                    makeprob.obj_ins,
                    early.motion_cost,
                    {**meta_of(rep_e, eb, how), "variant": 0},
                    "round_candidate",
                )
                (valid if rec["valid"] else invalid).append(rec)
            else:
                next_collisions = early_col
                col_env = env_e
        iters.append(
            {
                "iteration": count,
                "calls": [c.id for c in calls],
                "ready": [c.id for c in ready],
                "how": how,
                "early_rep": rep_e.id if rep_e else None,
                "late_rep": rep_l.id if rep_l else None,
                "valid": len(valid),
                "invalid": len(invalid),
                "colliding": bool(next_collisions),
            }
        )
        if valid:
            best = min(valid, key=lambda r: (r["pipeline_makespan"], r["candidate_id"]))
            pool.terminate(calls, "killed_run_end")
            return finish(ctx, "VALID", candidate=best, iterations=iters)
        if not next_collisions:
            pool.terminate(calls, "killed_run_end")
            return finish(
                ctx,
                "NO_REPAIRABLE_CONFLICT",
                "INVALID:" + "+".join(sorted({k for r in invalid for k, _ in r["violations"]})),
                iterations=iters,
            )
    pool.terminate(calls, "killed_run_end")
    return finish(ctx, "SEARCH_EXHAUSTED", "loop left without collisions", iterations=iters)
