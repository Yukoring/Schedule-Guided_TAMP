#!/usr/bin/env python3
"""Batch launcher for every experiment of the paper.

usage: run_batch.py plan|run|status|aggregate --exp EXP [--seeds 0 1 2] [--slots A:0,1,2,3 B:6,7,8,9] [--out DIR] [--budget 100]
 run_batch.py single --method ours --input inputs/struct/MP_s0.yaml --out DIR [--seed 0] [--cores 0,1,2,3]

EXP (paper tables)
 main 80 structured inputs (MP, SP, MT, ST) x {ours, cbtamp, tp, tp_sipp, tas_sipp, itags (MP/SP only)}; paper seeds 0 1 2
 ablation 80 structured inputs x {ours, no_fb, u, diverse_u, u_ad_pd, u_rt}; seed 0
 scalability 80 room inputs (R2J8..R8J32) + 20 map inputs (den312d, warehouse) x {ours, cbtamp, tp, tp_sipp, tas_sipp, itags}; seed 0
 tasfb structured 80 + rooms 80 + maps 20 x {tas_fb}; seed 0
 hw 10 hardware planning inputs (HW_MP/HW_MT s0..s4, 2.9 m arena) x {ours, u, cbtamp, tas_sipp}; seed 0; stack variant hw

Methods (names as in the paper)
 ours schedule-guided control layer (portfolio U/AD/PD/RT, beta retries, SIPP repair, 2 extra rounds, collision-region feedback)
 u, u_ad_pd, u_rt, diverse_u, no_fb ablation variants of ours
 cbtamp, tp, tp_sipp temporal-planning baselines on the shared stack
 tas_sipp joint CP-SAT task allocation and scheduling (gap 5 %/60 s, vertex occupancy, explicit parking) + one-shot SIPP
 tas_fb tas_sipp + collision-region feedback and rescheduling (CP-SAT cap 5 s escalating)
 itags ITAGS (C++ CP-SAT allocation search) + shared SIPP

Protocol (all experiments): budget 100 s per run from roadmap construction to validation, 4 dedicated cores per run, PRM seed = CP-SAT seed =
seed, final motion standard (endpoint retreat off, hand-over wait-aside off, parked robots occupy their last location). One slot = one
4-core worker; two slots run in parallel. Never put a second job on hyper-thread siblings of a running slot.
Outputs: OUT/manifest.csv, OUT/runs/<run_id>/ (full run record), OUT/output/dispatch_log.jsonl, OUT/output/runs.csv (aggregate)."""
import argparse, csv, json, os, sys, time, hashlib, subprocess, threading, platform, signal, shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STACK = ROOT / "common"
SD = ROOT / "baselines/tas_sipp"
TASFB = ROOT / "baselines/tas_fb"
ITAGS = ROOT / "baselines/itags"
INP = ROOT / "inputs"
BUDGET = 100.0
GRACE = 30.0
BASE = {
    "PYTHONHASHSEED": "0",
    "MPLBACKEND": "Agg",
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "PYTHONFAULTHANDLER": "1",
    "PYTHONUNBUFFERED": "1",
}
# paper name -> internal worker config / runner
METHODS = {
    "ours": "FULL",
    "u": "U",
    "u_ad_pd": "U_AD_PD",
    "u_rt": "U_RT",
    "no_fb": "NOFB",
    "diverse_u": "U_MATCH",
    "cbtamp": "CBTAMP",
    "tp": "PLANNER_ONLY",
    "tp_sipp": "TASK_PRIORITIZED",
    "tas_sipp": "JOINT_GAP",
    "tas_fb": "JOINT_FB",
    "itags": "ITAGS",
}
WORKER_METHODS = {"FULL", "U", "U_AD_PD", "U_RT", "NOFB", "U_MATCH", "CBTAMP", "PLANNER_ONLY", "TASK_PRIORITIZED"}
STRUCT = ["MP", "SP", "MT", "ST"]
ROOMS = ["R2J8", "R4J16", "R6J24", "R8J32"]
MAPS = ["den312d", "warehouse"]
EXPS = {
    "main": dict(groups=["struct"], methods=["ours", "cbtamp", "tp", "tp_sipp", "tas_sipp", "itags"], seeds=[0, 1, 2]),
    "ablation": dict(groups=["struct"], methods=["ours", "no_fb", "u", "diverse_u", "u_ad_pd", "u_rt"], seeds=[0]),
    "scalability": dict(
        groups=["rooms", "maps"], methods=["ours", "cbtamp", "tp", "tp_sipp", "tas_sipp", "itags"], seeds=[0]
    ),
    "tasfb": dict(groups=["struct", "rooms", "maps"], methods=["tas_fb"], seeds=[0]),
    "hw": dict(groups=["hw"], methods=["ours", "u", "cbtamp", "tas_sipp"], seeds=[0], variant="hw"),
}
now = lambda: time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def cases(group):
    if group == "struct":
        return [(f"{f}_s{s}", INP / "struct" / f"{f}_s{s}.yaml") for f in STRUCT for s in range(20)]
    if group == "rooms":
        return [(f"{f}_s{s}", INP / "scal" / f"{f}_s{s}.yaml") for f in ROOMS for s in range(20)]
    if group == "maps":
        return [(f"{f}_s{s}", INP / "maps" / f"{f}_s{s}.yaml") for f in MAPS for s in range(100, 110)]
    if group == "hw":
        return [(f"{f}_s{s}", INP / "hw/planner" / f"{f}_s{s}.yaml") for f in ["HW_MP", "HW_MT"] for s in range(5)]
    raise ValueError(group)


def variant_stack(name):
    """Build common + variant overlay on first use."""
    if not name:
        return STACK
    d = ROOT / "common/_build" / name
    if not (d / "run_worker.py").exists():
        shutil.copytree(
            STACK, d, ignore=shutil.ignore_patterns("variants", "_build", "__pycache__"), dirs_exist_ok=True
        )
        src = STACK / "variants" / name
        for f in src.rglob("*"):
            if f.is_file() and f.suffix != ".patch":
                t = d / f.relative_to(src)
                t.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(f, t)
    return d


def execute(job, cpus, slot, runs, py, budget):
    out = runs / job["run_id"]
    out.mkdir(parents=True, exist_ok=False)
    name = job["method"]
    m = METHODS[name]
    seed = int(job["seed"])
    env = dict(
        os.environ,
        **BASE,
        TMPDIR=str(out),
        ITAGS_CPSAT_SEED=str(seed),
    )
    start = time.monotonic()
    started = now()
    mission = "tool" if job["family"].split("_")[-1] in ("MT", "ST") else "precedence"
    cores = ",".join(map(str, cpus))
    if m == "JOINT_GAP":
        argv = [
            py,
            str(SD / "run_joint.py"),
            "--input",
            job["input_path"],
            "--mission",
            mission,
            "--output",
            str(out / "joint"),
            "--stack-root",
            str(variant_stack(job.get("variant"))),
            "--prm-seed",
            str(seed),
            "--cp-seed",
            str(seed),
            "--budget",
            str(budget),
            "--cores",
            cores,
            "--gap",
            "0.05",
            "--cp-max-s",
            "60.0",
            "--occupancy",
        ]
        cwd = str(SD)
    elif m == "JOINT_FB":
        argv = [
            py,
            str(TASFB / "run_joint_fb.py"),
            "--input",
            job["input_path"],
            "--mission",
            mission,
            "--output",
            str(out / "fb"),
            "--stack-root",
            str(variant_stack(job.get("variant"))),
            "--prm-seed",
            str(seed),
            "--cp-seed",
            str(seed),
            "--budget",
            str(budget),
            "--cores",
            cores,
            "--gap",
            "0.05",
            "--cp-max-s",
            "5",
        ]
        cwd = str(TASFB)
    elif m == "ITAGS":
        for folder in ["pddl_prob_plan", "results", "candidates", "calls"]:
            (out / folder).mkdir()
        (out / "pddl_domain").symlink_to(ITAGS / "code/pddl_domain", target_is_directory=True)
        argv = [
            py,
            str(ITAGS / "itags_worker.py"),
            "--run-dir",
            str(out),
            "--case",
            job["case"],
            "--config",
            "ITAGS",
            "--input",
            job["input_path"],
            "--budget",
            str(budget),
            "--cores",
            cores,
            "--start",
            repr(start),
            "--case-index",
            str(job["case_index"]),
            "--slot",
            slot,
            "--binary",
            str(ITAGS / "package/build_seed/itags_cpsat"),
        ]
        cwd = str(out)
    elif m in WORKER_METHODS:
        root = variant_stack("diverse_u" if m == "U_MATCH" else job.get("variant"))
        for folder in ["pddl_prob_plan", "results", "candidates", "calls"]:
            (out / folder).mkdir()
        for folder in ["pddl_domain", "planners"]:
            (out / folder).symlink_to(root / "code" / folder, target_is_directory=True)
        argv = [
            py,
            str(root / "run_worker.py"),
            "--run-dir",
            str(out),
            "--case",
            job["case"],
            "--config",
            m,
            "--seed",
            str(seed),
            "--input",
            job["input_path"],
            "--budget",
            str(budget),
            "--cores",
            cores,
            "--start",
            repr(start),
            "--case-index",
            str(job["case_index"]),
            "--slot",
            slot,
        ]
        cwd = str(out)
    else:
        raise ValueError(m)
    json.dump(
        {
            "job": job,
            "argv": argv,
            "env": {k: env[k] for k in env if k.startswith("ITAGS")},
            "cpus": cpus,
            "slot": slot,
            "start_utc": started,
        },
        open(out / "launch.json", "w"),
        indent=1,
    )
    with (out / "stdout.txt").open("w") as so, (out / "stderr.txt").open("w") as se:
        proc = subprocess.Popen(argv, stdout=so, stderr=se, env=env, start_new_session=True, cwd=cwd)
        timeout = False
        while proc.poll() is None:
            if time.monotonic() - start > budget + GRACE:
                timeout = True
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                proc.wait()
                break
            time.sleep(0.1)
    wall = time.monotonic() - start
    row = {
        "run_id": job["run_id"],
        "exp": job["exp"],
        "method": name,
        "config": m,
        "case": job["case"],
        "family": job["family"],
        "seed": seed,
        "slot": slot,
        "wall_s": round(wall, 3),
        "returncode": proc.returncode,
        "supervisor_timeout": timeout,
        "start_utc": started,
        "end_utc": now(),
    }
    try:
        if m in ("JOINT_GAP", "JOINT_FB"):
            r = json.load(open(out / ("joint" if m == "JOINT_GAP" else "fb") / "result.json"))
            row.update(
                {
                    "success": bool(r.get("solved")),
                    "end_reason": r.get("end_reason"),
                    "makespan": r.get("makespan"),
                    "runtime_s": r.get("runtime_s"),
                    "validation_end_s": r.get("validation_complete_at_s"),
                    "schedule_makespan": r.get("schedule_makespan"),
                    "stop_reason": r.get("stop_reason"),
                    "rounds": r.get("rounds"),
                    "first_round": json.dumps(r.get("first_round")) if r.get("first_round") else None,
                }
            )
        else:
            pr = json.load(open(out / "pipeline_return.json")) if (out / "pipeline_return.json").exists() else {}
            cands = (
                [json.load(open(f)) for f in sorted((out / "candidates").glob("*.json"))]
                if (out / "candidates").exists()
                else []
            )
            inc = [c for c in cands if c.get("valid") and c.get("validated_elapsed_s", 1e9) <= budget]
            best = min(inc, key=lambda c: c["trajectory_makespan"]) if inc else None
            if m == "ITAGS" and best is None and (out / "result.json").exists():
                r = json.load(open(out / "result.json"))
                row.update(
                    {
                        "success": bool(r.get("success")),
                        "end_reason": r.get("end_reason"),
                        "makespan": r.get("makespan_full_trajectory"),
                        "runtime_s": r.get("compute_time_s"),
                    }
                )
            else:
                row.update(
                    {
                        "success": best is not None,
                        "end_reason": pr.get("end_reason")
                        or ("BUDGET_EXHAUSTED" if wall >= budget else "DATA_MISSING"),
                        "makespan": best["trajectory_makespan"] if best else None,
                        "runtime_s": round(min(wall, budget), 3),
                        "validation_end_s": best.get("validated_elapsed_s") if best else None,
                        "first_valid_s": pr.get("first_valid_s"),
                        "selected_source": pr.get("selected_source"),
                        "geometry_iterations": pr.get("geometry_iterations"),
                    }
                )
    except Exception as e:
        row.update({"success": False, "end_reason": "ERROR", "error": repr(e)[:200]})
    json.dump(row, open(out / "result_row.json", "w"), indent=1, default=str)
    return row


def plan(a):
    e = EXPS[a.exp]
    seeds = a.seeds if a.seeds is not None else e["seeds"]
    rows = []
    for seed in seeds:
        for g in e["groups"]:
            for case, inp in cases(g):
                for m in e["methods"]:
                    if m == "itags" and case.split("_s")[0].split("_")[-1] in ("MT", "ST"):
                        continue
                    s = int(case.split("_s")[1])
                    rows.append(
                        {
                            "run_id": f"{a.exp.upper()}__{case}__{m}__seed{seed}",
                            "exp": a.exp,
                            "seed": seed,
                            "case": case,
                            "family": case.split("_s")[0],
                            "case_index": s,
                            "method": m,
                            "variant": e.get("variant", ""),
                            "input_path": str(inp),
                            "worker_slot": list(a.slot_map)[(len(rows) + s) % len(a.slot_map)],
                            "queue_rank": len(rows),
                        }
                    )
    a.out.mkdir(parents=True, exist_ok=True)
    with (a.out / "manifest.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("planned", len(rows), "runs ->", a.out / "manifest.csv")


def jobs(a):
    return [
        dict(r, case_index=int(r["case_index"]), queue_rank=int(r["queue_rank"]))
        for r in csv.DictReader((a.out / "manifest.csv").open())
    ]


def worker(a, slot, state):
    runs = a.out / "runs"
    runs.mkdir(exist_ok=True)
    (a.out / "output").mkdir(exist_ok=True)
    for job in sorted([j for j in jobs(a) if j["worker_slot"] == slot], key=lambda j: j["queue_rank"]):
        d = runs / job["run_id"]
        if d.exists():
            if (d / "result_row.json").exists():
                continue
            shutil.rmtree(d)  # partial run from an interrupted launcher: re-executed from scratch
        if (a.out / "STOP_REQUESTED").exists():
            state[slot] = "STOPPED"
            return
        row = execute(job, a.slot_map[slot], slot, runs, a.python, a.budget)
        with (a.out / "output/dispatch_log.jsonl").open("a") as f:
            f.write(
                json.dumps(
                    {
                        "utc": now(),
                        "slot": slot,
                        "run_id": row["run_id"],
                        "success": row.get("success"),
                        "end_reason": row.get("end_reason"),
                        "makespan": row.get("makespan"),
                        "wall_s": row["wall_s"],
                        "rc": row["returncode"],
                    }
                )
                + "\n"
            )
        print("RUN", row["run_id"], row.get("success"), row.get("end_reason"), row.get("makespan"), flush=True)
    state[slot] = "COMPLETED"


def run(a):
    if not (a.out / "manifest.csv").exists():
        plan(a)
    state = {}
    ths = [threading.Thread(target=worker, args=(a, s, state)) for s in a.slot_map]
    [t.start() for t in ths]
    [t.join() for t in ths]
    json.dump(
        {
            "status": "COMPLETED" if all(v == "COMPLETED" for v in state.values()) else "STOPPED",
            "completed": len(list((a.out / "runs").glob("*/result_row.json"))),
            "planned": len(jobs(a)),
            "utc": now(),
        },
        open(a.out / "status.json", "w"),
        indent=1,
    )
    aggregate(a)


def status(a):
    print(
        len(list((a.out / "runs").glob("*/result_row.json"))),
        "/",
        len(jobs(a)) if (a.out / "manifest.csv").exists() else "?",
    )


def aggregate(a):
    rows = [json.load(open(f)) for f in sorted((a.out / "runs").glob("*/result_row.json"))]
    if not rows:
        print("no rows")
        return
    cols = sorted(
        {k for r in rows for k in r},
        key=lambda k: (
            k
            not in (
                "run_id",
                "exp",
                "method",
                "case",
                "family",
                "seed",
                "success",
                "end_reason",
                "makespan",
                "runtime_s",
            ),
            k,
        ),
    )
    with (a.out / "output/runs.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        [w.writerow({k: ("" if r.get(k) is None else r[k]) for k in cols}) for r in rows]
    print("rows", len(rows), "->", a.out / "output/runs.csv")


def single(a):
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "runs").mkdir(exist_ok=True)
    (a.out / "output").mkdir(exist_ok=True)
    case = Path(a.input).stem
    job = {
        "run_id": f"SINGLE__{case}__{a.method}__seed{a.seed}",
        "exp": "single",
        "seed": a.seed,
        "case": case,
        "family": case.split("_s")[0],
        "case_index": int(case.split("_s")[1]) if "_s" in case else 0,
        "method": a.method,
        "variant": a.variant or "",
        "input_path": str(Path(a.input).resolve()),
    }
    if (a.out / "runs" / job["run_id"]).exists():
        shutil.rmtree(a.out / "runs" / job["run_id"])
    row = execute(job, [int(x) for x in a.cores.split(",")], "S", a.out / "runs", a.python, a.budget)
    print(
        json.dumps({k: row.get(k) for k in ("run_id", "success", "end_reason", "makespan", "runtime_s")}, default=str)
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("cmd", choices=["plan", "run", "status", "aggregate", "single"])
    p.add_argument("--exp", choices=list(EXPS))
    p.add_argument("--seeds", type=int, nargs="*", default=None)
    p.add_argument(
        "--slots", nargs="*", default=["A:0,1,2,3", "B:6,7,8,9"], help="slot name:cpus; one 4-core worker per slot"
    )
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--budget", type=float, default=BUDGET)
    p.add_argument("--python", default=sys.executable)
    p.add_argument("--method", choices=list(METHODS))
    p.add_argument("--input")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--cores", default="0,1,2,3")
    p.add_argument("--variant", default=None)
    a = p.parse_args()
    a.slot_map = {s.split(":")[0]: [int(x) for x in s.split(":")[1].split(",")] for s in a.slots}
    if a.out is not None:
        a.out = a.out.resolve()
    if a.cmd == "single":
        a.out = a.out or ROOT / "results/single"
        single(a)
    else:
        if not a.exp:
            raise SystemExit("--exp required")
        a.out = a.out or ROOT / "results" / a.exp
        {"plan": plan, "run": run, "status": status, "aggregate": aggregate}[a.cmd](a)
