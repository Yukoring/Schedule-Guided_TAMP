#!/usr/bin/env python3
"""One ITAGS run: roadmap, export, C++ allocation search with CP-SAT scheduling, SIPP motion stage, validation."""
import argparse, hashlib, json, os, random, resource, sys, time, traceback
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parent
COMMON = ROOT.parent.parent / "common"


def encode(x):
    if hasattr(x, "tolist"):
        return x.tolist()
    if hasattr(x, "item"):
        return x.item()
    if isinstance(x, set):
        return sorted(x)
    raise TypeError(type(x).__name__)


def save(p, x):
    p = Path(p)
    tmp = p.with_suffix(p.suffix + ".tmp")
    tmp.write_text(json.dumps(x, indent=1, default=encode) + "\n")
    tmp.replace(p)


def main():
    ap = argparse.ArgumentParser()
    for a in ["--run-dir", "--case", "--config", "--input", "--cores", "--slot"]:
        ap.add_argument(a, required=True)
    ap.add_argument("--budget", type=float, required=True)
    ap.add_argument("--start", type=float, required=True)
    ap.add_argument("--case-index", type=int, default=0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--binary", default=str(ROOT / "package/build/itags_cpsat"))
    args = ap.parse_args()
    out = Path(args.run_dir).resolve()
    os.chdir(out)
    cpus = [int(x) for x in args.cores.split(",")]
    os.sched_setaffinity(0, set(cpus))
    sys.path.insert(0, str(ROOT / "code"))
    sys.path.insert(0, str(COMMON))
    sys.path.insert(0, str(ROOT))
    os.environ.setdefault("MPLBACKEND", "Agg")
    import numpy as np
    from planner_pool import Budget, BudgetExpired
    import control_common as M, itags_adapter as A
    from pipelines import common
    from pipelines.common import load_plan
    from motion.final_validator import validate_final, _timed_events
    from motion.collision_vor import collision_check
    from motion import prioritized as P

    budget = Budget(args.start, args.budget)
    family = args.case.split("_s")[0]
    tool = family.split("_")[-1] in ("MT", "ST")
    (out / "candidates").mkdir(exist_ok=True)
    (out / "calls").mkdir(exist_ok=True)
    save(
        out / "configuration.json",
        {
            "protocol": "main",
            "method": "ITAGS-CPP-CP-SAT+SIPP" if not tool else "ITAGS-CPP-CP-SAT+SIPP-ToolExt",
            "case": args.case,
            "tool_input": tool,
            "budget_s": args.budget,
            "cores": cpus,
            "slot": args.slot,
            "binary": args.binary,
            "alpha": 0.5,
            "cpsat": {
                "time_scale_ms": 1,
                "relative_gap": 0.1,
                "secondary_gap": 0.0,
                "workers": 4,
                "per_refinement_cap_s": "min(10, remaining)",
            },
            "itags_policy": "first task-complete allocation (upstream)",
            "motion_policy": "SHARED motion layer: PrioritizedPlanningSolver (priority = robot id order, MAPD retreat), single attempt; no rotation, no precedence reordering, no release propagation, no wait-aside",
            "prm_seed": args.seed,
            "smooth": True,
            "python": sys.version,
        },
    )
    res_row = {
        "found": False,
        "adopted_candidate": None,
        "end_reason": None,
        "sub_reason": None,
        "return_ready_s": None,
        "validation_end_s": None,
        "makespan": None,
        "pipeline_makespan": None,
        "schedule_found": False,
        "valid_plan_found": False,
    }
    if tool:
        res_row.update(
            {
                "end_reason": "UNSUPPORTED",
                "sub_reason": "UNSUPPORTED_TOOL: portable-tool possession/transfer/hand-over semantics are not implemented in the ITAGS adapter (ToolExt); the input was not run as a non-tool problem",
                "elapsed_s": budget.elapsed(),
            }
        )
        save(out / "pipeline_return.json", res_row)
        save(out / "resource.json", {"elapsed_s": budget.elapsed(), "planner_pool": {"unreaped": [], "calls": 0}})
        return
    PRM_SEED = args.seed
    random.seed(PRM_SEED)
    np.random.seed(PRM_SEED)
    t_world = time.monotonic()
    try:
        world = common.build_world(args.input, False, False, PRM_SEED)
        world.prm.smooth = True
        ctx = M.Ctx(
            out,
            world,
            budget,
            None,
            False,
            args.case_index,
            lambda o, p: [],
            _timed_events,
            validate_final,
            collision_check,
            load_plan,
        )
        ctx.add("roadmap", world.construct_time)
        # ---- export + C++ search
        e0 = time.monotonic()
        mapping, norm, index = A.export_problem(world, out / "itags_input.json", max(0.05, budget.remaining()))
        ctx.add("export", time.monotonic() - e0)
        save(
            out / "itags_mapping.json",
            {
                "tasks": mapping,
                "normalisation": norm,
            },
        )
        budget.check()
        remaining = budget.remaining()
        s0 = time.monotonic()
        res = A.run_itags(
            args.binary, out / "itags_input.json", out / "itags_result.json", remaining, cpus, out / "itags_stdout.txt"
        )
        ctx.add("itags_search", time.monotonic() - s0)
        pr = res.get("_process", {})
        res_row.update(
            {
                "itags_status": res.get("status"),
                "itags_error": res.get("error"),
                "itags_elapsed_s": res.get("elapsed_seconds"),
                "itags_cpu_s": pr.get("cpu_s"),
                "itags_returncode": pr.get("returncode"),
                "itags_killed": pr.get("killed_by_adapter"),
                "search_statistics": res.get("search_statistics"),
                "scheduler_statistics": res.get("scheduler_statistics"),
                "schedule_makespan": res.get("makespan"),
                "schedule_best": res.get("schedule_best"),
                "schedule_worst": res.get("schedule_worst"),
                "cp_sat_calls": (res.get("scheduler_statistics") or {}).get("cp_sat_calls"),
                "budget_given_to_itags_s": remaining,
            }
        )
        ctx.event(
            type="itags_search",
            status=res.get("status"),
            elapsed=res.get("elapsed_seconds"),
            makespan=res.get("makespan"),
            stats=res.get("scheduler_statistics"),
        )
        if res.get("status") != "schedule_found_requires_motion_validation":
            st = res.get("status")
            end = {
                "no_schedule_returned": "NO_SCHEDULE",
                "wall_budget_exhausted": "TIMEOUT",
                "exception": "ERROR",
                "exited_without_result": "ERROR",
            }.get(st, "ERROR")
            if end == "ERROR" and (pr.get("killed_by_adapter") or budget.expired()):
                end = "TIMEOUT"
            res_row.update(
                {
                    "end_reason": end,
                    "sub_reason": "C++ ITAGS status %s%s"
                    % (st, (": " + str(res.get("error"))) if res.get("error") else ""),
                    "return_ready_s": budget.elapsed(),
                }
            )
            save(out / "pipeline_return.json", res_row)
            return
        res_row["schedule_found"] = True
        # motion stage: shared prioritized SIPP; the ITAGS schedule gives the assignment and per-robot order, its timing is not enforced
        samples = [tuple(p) for p in world.prm.task_samples]
        parsed, orders = A.parsed_plan_from_itags(res, mapping, world, samples)
        save(
            out / "itags_plan.json",
            {
                "allocation": res.get("allocation"),
                "timepoints": res.get("timepoints"),
                "robot_task_orders": {r: [list(x) for x in v] for r, v in orders.items()},
                "robot_plans": res.get("robot_plans"),
                "parsed_plan": parsed,
            },
        )
        names = list(world.env.robots_map)
        starts = {n: tuple(r.pos) for n, r in world.env.robots_map.items()}
        attempts = []
        final = None
        motion_fail = 0
        val_fail = 0
        budget.check()
        m0 = time.monotonic()
        solver = P.PrioritizedPlanningSolver(
            world.prm.task_roadmap, samples, {n: list(map(list, parsed[n])) for n in names}, starts=starts
        )
        rp, mk, tc = solver.find_solution()
        ctx.add("motion", time.monotonic() - m0)
        rec = {
            "order_index": 0,
            "rotation": names,
            "order": names,
            "iteration": 1,
            "motion_found": rp is not None,
            "elapsed_s": budget.elapsed(),
            "motion_layer": "shared PrioritizedPlanningSolver (single attempt)",
        }
        if rp is None:
            motion_fail += 1
            rec["outcome"] = "MOTION_FAILED"
            attempts.append(rec)
            ctx.event(type="sipp_attempt", **rec)
        else:
            plan = NS(path=rp, motion_cost=mk, total_cost=tc)
            col, _ = ctx.collide(rp, world.prm, [], None, 0)
            ctx.motion_candidate(plan, {"order_index": 0, "iteration": 1, "source_label": "ITAGS"}, col)
            if col:
                motion_fail += 1
                rec["outcome"] = "MOTION_FAILED_SHARED_CHECKER"
                rec["collisions"] = len(col)
                attempts.append(rec)
                ctx.event(type="sipp_attempt", **rec)
            else:
                c = ctx.record(
                    rp,
                    _obj_ins(world, mapping),
                    mk,
                    {
                        "order_index": 0,
                        "iteration": 1,
                        "source_label": "ITAGS",
                        "variant": 0,
                        "schedule_makespan": res.get("makespan"),
                    },
                    "itags_sipp",
                )
                rec["outcome"] = "VALID" if c["valid"] else "VALIDATION_FAILED"
                rec["candidate"] = c["candidate_id"]
                rec["violations"] = c["violations"][:8]
                attempts.append(rec)
                ctx.event(type="sipp_attempt", **rec)
                if c["valid"]:
                    final = c
                else:
                    val_fail += 1
        res_row.update(
            {
                "sipp_attempts": attempts,
                "orders_tried": len({a["order_index"] for a in attempts}),
                "motion_failures": motion_fail,
                "validation_failures": val_fail,
            }
        )
        if final:
            res_row.update(M.finish(ctx, "VALID", candidate=final))
            res_row["valid_plan_found"] = True
            res_row["schedule_found"] = True
        else:
            end = (
                "MOTION_FAILED"
                if motion_fail and not val_fail
                else ("VALIDATION_FAILED" if val_fail else "MOTION_FAILED")
            )
            res_row.update(
                {
                    "end_reason": end,
                    "sub_reason": "%d order(s) tried; motion failures %d; validation failures %d"
                    % (len({a["order_index"] for a in attempts}), motion_fail, val_fail),
                    "return_ready_s": budget.elapsed(),
                }
            )
    except BudgetExpired:
        best = ctx.best_valid() if "ctx" in dir() else None
        if best:
            res_row.update(M.finish(ctx, "VALID", "budget exhausted after a validated plan", candidate=best))
            res_row["valid_plan_found"] = True
        else:
            res_row.update(
                {
                    "end_reason": "TIMEOUT",
                    "sub_reason": "wall budget exhausted (%s)"
                    % ("during SIPP stage" if res_row.get("schedule_found") else "during export/search"),
                    "return_ready_s": None,
                }
            )
    except Exception:
        traceback.print_exc()
        save(out / "pipeline_error.json", {"traceback": traceback.format_exc(), "elapsed_s": budget.elapsed()})
        res_row.update({"end_reason": "ERROR", "sub_reason": traceback.format_exc().strip().splitlines()[-1][:160]})
    finally:
        rs = resource.getrusage(resource.RUSAGE_SELF)
        rc = resource.getrusage(resource.RUSAGE_CHILDREN)
        save(
            out / "resource.json",
            {
                "elapsed_s": budget.elapsed(),
                "self": {"utime_s": rs.ru_utime, "stime_s": rs.ru_stime, "maxrss_kb": rs.ru_maxrss},
                "children_reaped": {"utime_s": rc.ru_utime, "stime_s": rc.ru_stime, "maxrss_kb": rc.ru_maxrss},
                "planner_pool": {
                    "unreaped": [],
                    "calls": 0,
                    "active_wall_s": 0,
                    "paused_wall_s": 0,
                    "queued_wall_s": 0,
                    "cpu_time_s": 0,
                    "max_rss_kb": 0,
                    "end_how": {},
                    "metric_increase_calls": [],
                },
            },
        )
    res_row.update(
        {
            "elapsed_s": budget.elapsed(),
            "modules": ctx.modules if "ctx" in dir() else None,
            "temporal_calls": 0,
            "scheduler_calls": res_row.get("cp_sat_calls") or 0,
            "stored_candidates": ctx.n_cand if "ctx" in dir() else 0,
            "motion_candidates": ctx.n_motion if "ctx" in dir() else 0,
            "first_valid_s": ctx.first_valid_s if "ctx" in dir() else None,
            "config": args.config,
        }
    )
    save(out / "pipeline_return.json", res_row)


def _obj_ins(world, mapping):
    """Minimal object model for the validator."""
    robots = [NS(ind=n, action_duration=dict(zip(r.service, r.action_cost))) for n, r in world.env.robots_map.items()]
    wps = {}
    jobs = []
    tasks = {}
    for g, d in sorted(world.env.goals_map.items(), key=lambda kv: kv[1]["index"]):
        loc = tuple(world.goal_samples[d["index"]][0])
        w = "wp_%s" % g
        wps[w] = loc
        jobs.append(NS(ind=g, located=[w], order=[list(x) for x in d.get("order", [])]))
        for s in d["service"]:
            tasks.setdefault(s, []).append(g)
    return {
        "robot": robots,
        "job": jobs,
        "task": [NS(ind=s, loc=gs) for s, gs in tasks.items()],
        "waypoint": [NS(ind=w, loc=l) for w, l in wps.items()],
    }


if __name__ == "__main__":
    main()
