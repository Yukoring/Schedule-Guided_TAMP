#!/usr/bin/env python3
"""One run: one input, one method, 100 s budget, four cores. Builds the roadmap, runs the method loop and records calls, candidates, validation and feedback."""
import argparse, hashlib, json, os, random, resource, sys, time, traceback
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parent
OURS = ("FULL", "NOFB", "U", "U_AD_PD", "U_RT", "U_MATCH")
BASELINE = ("CBTAMP", "PLANNER_ONLY", "TASK_PRIORITIZED")


def encode(x):
    if hasattr(x, "tolist"):
        return x.tolist()
    if hasattr(x, "item"):
        return x.item()
    if isinstance(x, set):
        return sorted(x)
    raise TypeError(type(x).__name__)


def save(path, x):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(x, indent=1, default=encode) + "\n")
    tmp.replace(path)


def append(path, x):
    with Path(path).open("a") as f:
        f.write(json.dumps(x, default=encode) + "\n")


def main():
    ap = argparse.ArgumentParser()
    for a in ["--run-dir", "--case", "--config", "--input", "--cores", "--slot"]:
        ap.add_argument(a, required=True)
    ap.add_argument("--budget", type=float, required=True)
    ap.add_argument("--start", type=float, required=True)
    ap.add_argument("--case-index", type=int, required=True)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    out = Path(args.run_dir).resolve()
    os.chdir(out)
    cpus = [int(x) for x in args.cores.split(",")]
    os.sched_setaffinity(0, set(cpus))
    sys.path.insert(0, str(ROOT / "code"))
    sys.path.insert(0, str(ROOT))
    os.environ.setdefault("MPLBACKEND", "Agg")
    import numpy as np
    import planner_pool, cpsat_limits, methods as M
    from planner_pool import Budget, PlannerPool, BudgetExpired
    from pipelines import common
    from pipelines.common import load_plan
    from motion.final_validator import validate_final, _timed_events
    from motion.collision_vor import collision_check
    from motion.prioritized import PrioritizedPlanningSolver
    from world.prm import PRMPlanning
    from vrp import cpsat_module
    from task_planning.pddl_vrp_generator import Planner, read_domain
    from task_planning.pddlgenerator_vor import PDDLProblemGenerator
    import tool_adapter, baseline_tool_adapter as bta

    assert cpsat_module.VRP_TIME_LIMIT == 5
    budget = Budget(args.start, args.budget)
    cfg = args.config
    family = args.case.split("_s")[0]
    tool = family.split("_")[-1] in ("MT", "ST")
    tool_errors = lambda obj, paths: tool_adapter.validate_tools(NS(tool=True, obj_ins=obj), paths)
    dom_dir = ROOT / "code/pddl_domain"
    if cfg in OURS:
        if tool:
            for n in ["ours_domain_early.pddl", "ours_domain_late.pddl"]:
                (out / n).write_text(tool_adapter.make_domain(dom_dir / n))
            domains = {"early": str(out / "ours_domain_early.pddl"), "late": str(out / "ours_domain_late.pddl")}
            gen_cls = tool_adapter.ToolPlanner
        else:
            domains = {"early": str(dom_dir / "ours_domain_early.pddl"), "late": str(dom_dir / "ours_domain_late.pddl")}
            gen_cls = Planner
    else:
        if tool:
            for n in ["tp_domain_early.pddl", "tp_domain_late.pddl"]:
                (out / n).write_text(bta.make_baseline_domain(dom_dir / n, out))
            domains = {"early": str(out / "tp_domain_early.pddl"), "late": str(out / "tp_domain_late.pddl")}
            gen_cls = bta.ToolProblemGenerator
        else:
            domains = {"early": str(dom_dir / "tp_domain_early.pddl"), "late": str(dom_dir / "tp_domain_late.pddl")}
            gen_cls = PDDLProblemGenerator
    policy_log = []
    cpsat_limits.install(cpsat_module, budget, policy_log)
    orig_solver = cpsat_module.CpSatModule.solver
    sched = [0]

    def logged_solver(self):
        t = budget.elapsed()
        res = orig_solver(self)
        done = budget.elapsed()
        sched[0] += 1
        stats = dict(getattr(self, "last_stats", {}))
        dl = stats.pop("deadline_guidance", None)
        info = res[2] if isinstance(res, tuple) and len(res) > 2 else {}
        gb = info.get("goal_bound") if isinstance(info, dict) else None
        rec = {
            "index": sched[0] - 1,
            "update_reason": "initial" if sched[0] == 1 else "collision_feedback (regions/transients updated)",
            "start_elapsed_s": t,
            "end_elapsed_s": done,
            "solve_s": done - t,
            "cp_status": stats.get("cp_status"),
            "cp_wall_s": stats.get("cp_wall_s"),
            "T_hat_s": (dl or {}).get("T_hat_s"),
            "beta_s": (dl or {}).get("beta_s"),
            "margin_scale": getattr(self, "margin_scale", None),
            "deadline_tasks": [[g, x[0], x[2]] for g, v in (gb or {}).items() for x in v],
            "til_count": sum(len(v) for v in (gb or {}).values()),
            "region_count": stats.get("region_count"),
            "active_region_count": stats.get("active_region_count"),
            "exclusion_count": stats.get("region_exclusion_count"),
            "service_completion_cs": [
                (c["goal"], c["task"], c["robot"], c["start_cs"], c["end_cs"]) for c in (dl or {}).get("selected", [])
            ],
            "assignment": info.get("robot_task") if isinstance(info, dict) else None,
            "tours": info.get("tours") if isinstance(info, dict) else None,
            "policy": policy_log[-1] if policy_log else None,
        }
        append(out / "schedules.jsonl", rec)
        append(
            out / "scheduler_calls.jsonl",
            {
                "index": sched[0] - 1,
                "start_elapsed_s": t,
                "end_elapsed_s": done,
                "solve_s": done - t,
                "margin_scale": getattr(self, "margin_scale", None),
                "relax_ms": res[1] if isinstance(res, tuple) else None,
                "dispatch_present": res[0] is not None if isinstance(res, tuple) else None,
                "stats": stats,
                "policy": policy_log[-1] if policy_log else None,
                "assignment": info.get("robot_task") if isinstance(info, dict) else None,
                "tours": info.get("tours") if isinstance(info, dict) else None,
                "deadlines_exported": (
                    {j: [(a, b / 100.0, c) for a, b, c in v] for j, v in info.get("goal_bound", {}).items()}
                    if isinstance(info, dict) and info.get("goal_bound")
                    else None
                ),
                "deadline_guidance": dl,
                "regions": stats.get("region_count"),
                "transient_regions": (
                    len(getattr(self, "regions", [])) - stats.get("region_count", 0) if False else None
                ),
            },
        )
        return res

    cpsat_module.CpSatModule.solver = logged_solver
    PRM_SEED = args.seed
    random.seed(PRM_SEED)
    np.random.seed(PRM_SEED)
    save(
        out / "configuration.json",
        {
            "case": args.case,
            "config": cfg,
            "tool_extension": tool,
            "input": args.input,
            "budget_s": args.budget,
            "cores": cpus,
            "slot": args.slot,
            "case_index": args.case_index,
            "python": sys.version,
            "planner_profiles": {
                "phase_A": "U and temporal baselines: -c -S; AD/PD/RT: -n",
                "phase_B": "U and temporal baselines: -n",
                "beta_retries": "AD/PD: -n",
                "extension": "preserved processes of both phases, same PID",
            },
            "base_s": 5.0,
            "quantum_s": 1.0,
            "max_active_calls": 4,
            "cpsat": {"workers": 4, "seed": 0, "time_limit": "min(5 s, remaining budget)"},
            "guidance_consts": M.GCONST,
            "active_guidance": M.ACTIVE.get(cfg),
            "rotation": "[U,AD,PD,RT] rotated by (case_index+geometry_iteration) mod 4, early pass then late pass",
            "beta": "d_j=e_j+beta*(k_j+1), beta0=0.6 s, scale 1->4->16 (0.6,2.4,9.6 s)" if cfg in OURS else None,
            "domains": domains,
            "generator": gen_cls.__name__,
            "prm_smooth_initial": True,
            "temporal_planner": str(ROOT / planner_pool.BINARY),
        },
    )
    pool = PlannerPool(out, cpus, budget, ROOT / planner_pool.BINARY)
    res = None
    world = None
    try:
        world = common.build_world(args.input, False, False, PRM_SEED)
        world.prm.smooth = True
        save(
            out / "roadmap.json",
            {
                "construction_s": world.construct_time,
                "prm_seed": PRM_SEED,
                "requested_samples": world.env.init_num_of_samples,
                "actual_initial_vertices": {str(k): len(v) for k, v in world.init_samples.items()},
                "task_samples": len(world.prm.task_samples),
                "goals": len(world.env.goals_map),
                "robots": len(world.env.robots_map),
                "world_ready_elapsed_s": budget.elapsed(),
                "tool_initial_position": list(world.goal_samples[0][0]) if tool else None,
            },
        )
        append(
            out / "prm_log.jsonl",
            {"index": 0, "kind": "initial", "elapsed_s": budget.elapsed(), "smooth": True},
        )
        ctx = M.Ctx(
            out,
            world,
            budget,
            pool,
            tool,
            args.case_index,
            tool_errors,
            _timed_events,
            validate_final,
            collision_check,
            load_plan,
        )
        ctx.add("roadmap", world.construct_time)

        def log_prm(n, samples, roadmap, goals, prm):
            append(
                out / "prm_log.jsonl",
                {
                    "index": n,
                    "kind": "resample",
                    "elapsed_s": budget.elapsed(),
                    "smooth": getattr(prm, "smooth", None),
                    "vertices": {str(k): len(v) for k, v in samples.items()},
                },
            )

        if cfg in OURS:
            res = M.ours_loop(ctx, cfg, gen_cls, domains, cpsat_module.CpSatRegionModule, read_domain)
        elif cfg == "CBTAMP":
            res = M.cbtamp_loop(ctx, gen_cls, domains, read_domain)
        elif cfg == "PLANNER_ONLY":
            res = M.planner_only_loop(ctx, gen_cls, domains, read_domain, PRMPlanning, log_prm)
        elif cfg == "TASK_PRIORITIZED":
            if tool:
                # tool missions: hand-over propagation adapter; a budget stop inside it is reported as BUDGET_EXHAUSTED
                holder = []

                def factory(rm, s, plan, starts):
                    sv = bta.ToolPrioritizedSolver(
                        rm,
                        s,
                        plan,
                        bta.ToolProblemGenerator.current.tool_events,
                        starts=starts,
                        budget_check=budget.expired,
                    )
                    holder.append(sv)
                    return sv

                res = M.prioritized_loop(ctx, gen_cls, domains, read_domain, factory)
                if holder:
                    sv = holder[-1]
                    save(
                        out / "handover.json",
                        {
                            "adapter": bta.ADAPTER_VERSION,
                            "rounds": sv.handover_rounds,
                            "failure": sv.handover_failure,
                            "history": sv.handover_history,
                            "waits_final_pass": sv.handover_waits,
                            "pickup_predecessors": {str(k): v for k, v in sv.pred.items()},
                            "tool_events": sv.tool_events,
                            "priority_order": sv.names,
                            "idle": sorted(sv.idle),
                        },
                    )
                    if (
                        res is not None
                        and sv.handover_failure
                        and str(sv.handover_failure.get("cause", "")).startswith("budget exhausted")
                    ):
                        res["end_reason"] = "BUDGET_EXHAUSTED"
                        res["sub_reason"] = (
                            "wall budget exhausted during hand-over propagation (round %d)" % sv.handover_rounds
                        )
                    elif res is not None and sv.handover_failure and res.get("end_reason") == "NO_PATH":
                        res["sub_reason"] = "hand-over propagation failed: " + str(sv.handover_failure.get("cause"))
            else:
                factory = lambda rm, s, plan, starts: PrioritizedPlanningSolver(rm, s, plan, starts=starts)
                res = M.prioritized_loop(ctx, gen_cls, domains, read_domain, factory)
        else:
            raise ValueError(cfg)
    except BudgetExpired:
        best = ctx.best_valid() if "ctx" in dir() else None
        if best is not None:
            res = M.finish(
                ctx,
                "VALID",
                "budget exhausted after a validated incumbent (phase B or extension not completed); the incumbent validated within the budget is returned",
                candidate=best,
                budget_expired=True,
            )
        else:
            res = {
                "found": False,
                "adopted_candidate": None,
                "end_reason": "BUDGET_EXHAUSTED",
                "sub_reason": "wall budget %.0f s exhausted inside the control loop" % args.budget,
                "return_ready_s": None,
                "validation_end_s": None,
                "makespan": None,
            }
    except Exception:
        traceback.print_exc()
        pool.kill_all()
        save(out / "pipeline_error.json", {"traceback": traceback.format_exc(), "elapsed_s": budget.elapsed()})
        raise
    finally:
        pool.kill_all()
        with (out / "calls.jsonl").open("w") as f:
            for c in pool.calls:
                f.write(json.dumps(c.record(), default=encode) + "\n")
            for ev in (json.loads(l) for l in (out / "events.jsonl").open()) if (out / "events.jsonl").exists() else []:
                if ev.get("type") == "guidance_unavailable":
                    for ph in ev.get("calls_not_created", []):
                        f.write(
                            json.dumps(
                                {
                                    "call_id": None,
                                    "geometry_iteration": ev["iteration"],
                                    "beta_attempt": 0,
                                    **ph,
                                    "cp_status": ev.get("cp_status"),
                                    "t": ev["t"],
                                }
                            )
                            + "\n"
                        )
        rs = resource.getrusage(resource.RUSAGE_SELF)
        rc = resource.getrusage(resource.RUSAGE_CHILDREN)
        save(
            out / "resource.json",
            {
                "elapsed_s": budget.elapsed(),
                "self": {"utime_s": rs.ru_utime, "stime_s": rs.ru_stime, "maxrss_kb": rs.ru_maxrss},
                "children_reaped": {"utime_s": rc.ru_utime, "stime_s": rc.ru_stime, "maxrss_kb": rc.ru_maxrss},
                "planner_pool": pool.summary(),
                "affinity": sorted(os.sched_getaffinity(0)),
            },
        )
    if res is not None:
        res.update(
            {
                "elapsed_s": budget.elapsed(),
                "modules": ctx.modules if "ctx" in dir() else None,
                "temporal_calls": len(pool.calls),
                "scheduler_calls": sched[0],
                "stored_candidates": ctx.n_cand if "ctx" in dir() else 0,
                "motion_candidates": ctx.n_motion if "ctx" in dir() else 0,
                "first_valid_s": ctx.first_valid_s if "ctx" in dir() else None,
                "planner_pool": pool.summary(),
                "config": cfg,
                "tool_extension": tool,
            }
        )
        save(out / "pipeline_return.json", res)


if __name__ == "__main__":
    main()
