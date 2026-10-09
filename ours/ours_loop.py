#!/usr/bin/env python3
"""Ours: the schedule-guided control layer (Algorithm 1). Temporal planner calls with schedule guidance (AD, PD, RT) and without (U), beta retries, SIPP refinement and repair, validation, extra rounds after a valid plan, collision feedback to both planning models."""
import copy, hashlib, json, time
from pathlib import Path
from types import SimpleNamespace as NS
from control_common import *
from control_common import _single_problem_round


def ours_loop(ctx, config, gen_cls, domains, oracle_cls, read_domain):
    """One run of Ours. Each round runs guided and unguided planner calls in two phases, refines the candidates with SIPP, validates them, keeps the best valid plan and feeds the conflicts of colliding candidates back as collision regions; when the guided calls return nothing, beta is increased and they are retried."""
    env, prm = ctx.world.env, ctx.world.prm
    budget = ctx.budget
    pool = ctx.pool
    active = ACTIVE[config]
    d_name = read_domain(domains["early"])
    s0 = time.monotonic()
    task_prob = gen_cls(env, prm, d_name, "problem.pddl")
    task_prob.env_instance(pots=0)
    ctx.add("setup", time.monotonic() - s0)
    sched_prob = task_prob
    margin_scale = 1.0
    col_env = []
    tmem = []
    count = 0
    beta_events = []
    pending_beta = None
    iters = []
    incumbent = None
    extra_used = 0
    extra_events = []
    while True:
        budget.check()
        it = {"iteration": count, "margin_scale_at_start": margin_scale, "rounds": []}
        it_coll = []
        task_prob.transient_regions = [m["samples"] for m in tmem] if tmem else []
        v0 = time.monotonic()
        oracle = oracle_cls(sched_prob, margin_scale, region_exclusion=True)
        _, relax_ms, vrp_info = oracle.solver()
        ctx.add("scheduler", time.monotonic() - v0)
        if pending_beta is not None:
            pending_beta["applied"] = True
            pending_beta["applied_iteration"] = count
            pending_beta["applied_margin_scale"] = margin_scale
            beta_events.append(pending_beta)
            ctx.event(**{**pending_beta, "type": "beta_applied"})
            pending_beta = None
        avail = set(["U"])
        if vrp_info.get("goal_bound"):
            avail |= {"AD", "PD"}
        if any(vrp_info.get("robot_task", {}).values()):
            avail.add("RT")
        unavailable = [g for g in active if g not in avail]
        if unavailable:
            ctx.event(
                type="guidance_unavailable",
                iteration=count,
                configs=unavailable,
                cp_status=oracle.last_stats.get("cp_status"),
                calls_not_created=[
                    {"guidance": g, "release": rel, "profile": "n", "phase": "A", "call_status": "GUIDANCE_UNAVAILABLE"}
                    for rel in ("early", "late")
                    for g in unavailable
                ],
            )
        rot = (ctx.case_index + count) % 4
        order = [g for g in ROTATION[rot:] + ROTATION[:rot] if g in active and g in avail]
        it.update(
            {
                "rotation": rot,
                "order": order,
                "guidance_unavailable": unavailable,
                "relax_ms": relax_ms,
                "cp_status": oracle.last_stats.get("cp_status"),
            }
        )
        g0 = time.monotonic()
        p_name = task_prob.generate_problem(vrp_info, count, consts=[GCONST[g] for g in order])
        ctx.add("pddl", time.monotonic() - g0)

        def mk(g, rel, flags, phase, attempt=0, pn=None, ms=None):
            ms = ms or margin_scale
            return pool.new_call(
                domains[rel],
                (pn or p_name)[GCONST[g]],
                flags=flags,
                profile="cS" if flags == CS else "n",
                guidance=g,
                release=rel,
                phase=phase,
                geometry_iteration=count,
                beta_attempt=attempt,
                margin_scale=ms,
                beta_s=round(0.6 * ms, 3),
                const=GCONST[g],
            )

        all_calls = []
        beta_attempt = 0
        dr_cand_seen = False
        outcome = None

        def judge(cands_calls, phase):
            """Build every ready candidate, check collisions, validate the collision-free ones."""
            nonlocal dr_cand_seen
            loaded = []
            parser_errors = 0
            for c in cands_calls:
                plan, b = ctx.load(task_prob, c, c.meta["guidance"], count)
                if plan is None:
                    parser_errors += 1
                    continue
                if c.meta["guidance"] in ("AD", "PD"):
                    dr_cand_seen = True
                loaded.append((c, plan, b))
            clean, coll = [], []
            for c, plan, b in loaded:
                col, _ = ctx.collide(plan.path, prm, col_env, None, count)
                ctx.motion_candidate(
                    plan,
                    {
                        "call_id": c.id,
                        "source_label": label(c.meta),
                        "guidance": c.meta["guidance"],
                        "profile": c.meta["profile"],
                        "release": c.meta["release"],
                        "iteration": count,
                        "beta_attempt": c.meta["beta_attempt"],
                        "block": b["index"],
                        "phase": phase,
                    },
                    col,
                )
                (coll if col else clean).append((c, plan, col, b))
            valid = []
            invalid = []
            repaired = set()
            if coll:
                import sipp_repair

                for c, plan, col, b in sorted(coll, key=lambda x: x[1].motion_cost)[:SIPP_REPAIR_MAX]:
                    rec = sipp_repair.repair(
                        ctx,
                        task_prob,
                        prm,
                        c,
                        b,
                        {
                            "call_id": c.id,
                            "source_label": label(c.meta) + "+sipp",
                            "guidance": c.meta["guidance"],
                            "profile": c.meta["profile"],
                            "release": c.meta["release"],
                            "iteration": count,
                            "beta_attempt": c.meta["beta_attempt"],
                            "block": b["index"],
                            "variant": GCONST[c.meta["guidance"]],
                            "phase": phase,
                            "repair": "sipp",
                            "naive_pipeline_makespan": plan.motion_cost,
                        },
                        count,
                        col_env,
                    )
                    if rec is None:
                        continue
                    (valid if rec["valid"] else invalid).append(rec)
                    if rec["valid"]:
                        repaired.add(c.id)
            it_coll.extend([x for x in coll if x[0].id not in repaired])
            for c, plan, col, b in clean:
                rec = ctx.record(
                    plan.path,
                    task_prob.obj_ins,
                    plan.motion_cost,
                    {
                        "call_id": c.id,
                        "source_label": label(c.meta),
                        "guidance": c.meta["guidance"],
                        "profile": c.meta["profile"],
                        "release": c.meta["release"],
                        "iteration": count,
                        "beta_attempt": c.meta["beta_attempt"],
                        "block": b["index"],
                        "variant": GCONST[c.meta["guidance"]],
                        "phase": phase,
                        "deadline_vector": (
                            vrp_info.get("goal_bound")
                            if c.meta["guidance"] in ("AD", "PD") and c.meta["beta_attempt"] == 0
                            else None
                        ),
                    },
                    "round_candidate",
                )
                (valid if rec["valid"] else invalid).append(rec)
            it["rounds"].append(
                {
                    "phase": phase,
                    "beta_attempt": beta_attempt,
                    "calls": [c.id for c in cands_calls],
                    "loaded": len(loaded),
                    "parser_errors": parser_errors,
                    "clean": len(clean),
                    "colliding": len(coll),
                    "valid": len(valid),
                    "invalid": len(invalid),
                    "sipp_repaired_valid": len(repaired),
                }
            )
            return loaded, clean, coll, valid, invalid, parser_errors

        def decide(loaded, coll, valid, invalid):
            if valid:
                return (
                    "VALID",
                    min(valid, key=lambda r: (r["trajectory_makespan"], r["pipeline_makespan"], r["candidate_id"])),
                )
            if coll:
                return ("FEEDBACK", coll)
            if loaded:
                return ("NO_REPAIRABLE_CONFLICT", invalid)
            return None

        # phase A: U unguided, AD/PD/RT guided (early then late)
        A = [mk(g, rel, CS if g == "U" else N, "A") for rel in ("early", "late") for g in order]
        all_calls += A
        pool.base_round(A)
        la, ca, colA, vA, iA, _ = judge([c for c in A if c.has_block()], "A")
        # phase B: U with the second planner profile (early, late)
        B = [mk("U", rel, N, "B") for rel in ("early", "late")] if "U" in order else []
        all_calls += B
        if B:
            pool.base_round(B)
        lb, cb, colB, vB, iB, _ = judge([c for c in B if c.has_block()], "B")
        outcome = decide(la + lb, colA + colB, vA + vB, iA + iB)
        while outcome is None:
            dr = [g for g in order if g in ("AD", "PD")]
            if dr and margin_scale < BETA_MAX and not dr_cand_seen:
                pool.terminate([c for c in all_calls if c.meta["guidance"] in ("AD", "PD")], "killed_beta_retry")
                margin_scale = min(BETA_MAX, margin_scale * 4)
                beta_attempt += 1
                gb, beta_s = rebuild_goal_bound(oracle, margin_scale)
                info2 = dict(vrp_info)
                info2["goal_bound"] = gb
                keep = task_prob.pFilename
                task_prob.pFilename = "problem_beta%d.pddl" % beta_attempt
                g0 = time.monotonic()
                p2 = task_prob.generate_problem(info2, count, consts=[GCONST[g] for g in dr])
                ctx.add("pddl", time.monotonic() - g0)
                task_prob.pFilename = keep
                new = [
                    mk(g, rel, N, "beta", beta_attempt, pn=p2, ms=margin_scale) for rel in ("early", "late") for g in dr
                ]
                ev = {
                    "type": "beta_increase",
                    "trigger": "no parseable candidate from any call of the base round (beta attempt)",
                    "iteration": count,
                    "beta_attempt": beta_attempt,
                    "margin_scale": margin_scale,
                    "beta_s": beta_s,
                    "call_ids": [c.id for c in new],
                    "applied": True,
                    "scheduler_resolved": False,
                }
                beta_events.append(ev)
                ctx.event(**ev)
                all_calls += new
                pool.base_round(new)
                l, c_, co, v, i, _ = judge([c for c in new if c.has_block()], "beta%d" % beta_attempt)
                outcome = decide(l, co, v, i)
                continue
            arrivals, reason = pool.extend(all_calls)
            if reason == "candidate":
                l, c_, co, v, i, _ = judge(arrivals, "extension")
                outcome = decide(l, co, v, i)
                continue  # parse failure of the arriving block: keep extending
            outcome = ("EXHAUSTED", None)
        it["beta_attempts"] = beta_attempt
        it["calls"] = [c.id for c in all_calls]
        it["outcome"] = outcome[0]
        iters.append(it)
        ctx.event(type="iteration_end", **{k: v for k, v in it.items() if k != "rounds"})
        if outcome[0] == "VALID" and (
            incumbent is None or outcome[1]["trajectory_makespan"] < incumbent["trajectory_makespan"] - TIE
        ):
            incumbent = outcome[1]
        if config == "NOFB":
            # No FB: stop after the first round
            pool.terminate(all_calls, "killed_run_end")
            if outcome[0] == "NO_REPAIRABLE_CONFLICT":
                kinds = sorted({k for r in outcome[1] for k, _ in r["violations"]})
                return finish(
                    ctx,
                    "NO_REPAIRABLE_CONFLICT",
                    "INVALID:" + "+".join(kinds),
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                )
            if outcome[0] == "EXHAUSTED":
                errs = [c.id for c in all_calls if error_exit(c)]
                return finish(
                    ctx,
                    "PLANNER_ERROR" if errs and len(errs) == len(all_calls) else "SEARCH_EXHAUSTED",
                    ("calls_with_error_exit=%d" % len(errs)) if errs else None,
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                )
            if incumbent is not None:
                return finish(
                    ctx,
                    "VALID",
                    "feedback disabled: incumbent of the first round",
                    candidate=incumbent,
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                    extra_rounds_used=0,
                    extra_round_events=[],
                )
            return finish(
                ctx,
                "SEARCH_EXHAUSTED",
                "feedback disabled: no validated candidate in the single round (%d colliding)" % len(outcome[1]),
                iterations=iters,
                beta_events=beta_events,
                geometry_iterations=count + 1,
                final_margin_scale=margin_scale,
            )
        if incumbent is not None:
            shorter = [x for x in it_coll if x[1].motion_cost < incumbent["trajectory_makespan"] - TIE]
            if EXTRA_ROUNDS and extra_used < EXTRA_ROUNDS and shorter and outcome[0] in ("VALID", "FEEDBACK"):
                extra_used += 1
                best_short = min(shorter, key=lambda x: x[1].motion_cost)
                ev = {
                    "type": "extra_round",
                    "iteration": count,
                    "extra_round": extra_used,
                    "incumbent_candidate": incumbent["candidate_id"],
                    "incumbent_makespan": incumbent["trajectory_makespan"],
                    "incumbent_source": incumbent.get("source_label"),
                    "shorter_colliding": len(shorter),
                    "best_colliding_source": label(best_short[0].meta),
                    "best_colliding_pipeline_makespan": best_short[1].motion_cost,
                    "round_outcome": outcome[0],
                }
                extra_events.append(ev)
                ctx.event(**ev)
                outcome = ("FEEDBACK", shorter)
            else:
                pool.terminate(all_calls, "killed_run_end")
                sub = (
                    None
                    if not extra_used
                    else "incumbent returned after %d extra round(s) (%s)"
                    % (
                        extra_used,
                        (
                            "no shorter colliding candidate left"
                            if not shorter
                            else (
                                "extra-round limit %d reached" % EXTRA_ROUNDS
                                if extra_used >= EXTRA_ROUNDS
                                else "round ended with " + outcome[0]
                            )
                        ),
                    )
                )
                return finish(
                    ctx,
                    "VALID",
                    sub,
                    candidate=incumbent,
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                    extra_rounds_used=extra_used,
                    extra_round_events=extra_events,
                )
        if outcome[0] == "NO_REPAIRABLE_CONFLICT":
            kinds = sorted({k for r in outcome[1] for k, _ in r["violations"]})
            pool.terminate(all_calls, "killed_run_end")
            return finish(
                ctx,
                "NO_REPAIRABLE_CONFLICT",
                "INVALID:" + "+".join(kinds),
                iterations=iters,
                beta_events=beta_events,
                geometry_iterations=count + 1,
                final_margin_scale=margin_scale,
            )
        if outcome[0] == "EXHAUSTED":
            errs = [c.id for c in all_calls if error_exit(c)]
            pool.terminate(all_calls, "killed_run_end")
            return finish(
                ctx,
                "PLANNER_ERROR" if errs and len(errs) == len(all_calls) else "SEARCH_EXHAUSTED",
                ("calls_with_error_exit=%d" % len(errs)) if errs else None,
                iterations=iters,
                beta_events=beta_events,
                geometry_iterations=count + 1,
                final_margin_scale=margin_scale,
            )
        # feedback: the colliding candidate with the smallest makespan supplies the collision regions
        coll = outcome[1]
        c, plan, col, b = min(coll, key=lambda x: x[1].motion_cost)
        _, sel_env = ctx.collide(plan.path, prm, col_env, tmem, count)
        if sel_env != -1:
            col_env = sel_env
        s0 = time.monotonic()
        task_prob.env_update(col_env)
        ctx.add("setup", time.monotonic() - s0)
        ctx.event(
            type="feedback",
            iteration=count,
            repaired_call=c.id,
            source=label(c.meta),
            makespan=plan.motion_cost,
            regions=len(col_env) if isinstance(col_env, list) else None,
            transient=len(tmem),
        )
        if [g for g in order if g in ("AD", "PD")] and not dr_cand_seen and margin_scale < BETA_MAX:
            margin_scale = min(BETA_MAX, margin_scale * 4)
            pending_beta = {
                "type": "beta_increase",
                "trigger": "AD/PD produced no parseable candidate in this iteration; next iteration goes on with U/RT feedback",
                "iteration": count,
                "margin_scale": margin_scale,
                "beta_s": round(0.6 * margin_scale, 3),
                "applied": False,
            }
            ctx.event(**{**pending_beta, "type": "beta_increase_pending"})
        pool.terminate(all_calls, "killed_next_iteration")
        count += 1
