#!/usr/bin/env python3
"""TAS+FB: TAS+SIPP with collision feedback. Each round: CP-SAT (cap 5 s, doubled while no schedule is found) -> one SIPP attempt -> validation; on failure the schedule is replayed on the roadmap, its conflicts become collision regions, and the next CP-SAT call treats them as shared resources. Stops on a valid plan, when no conflict can be extracted, or at the budget."""
import argparse, hashlib, json, math, os, signal, subprocess, sys, time, traceback, heapq, random
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from run_joint import enc, save, sha, build_instance, export_sipp

SCALE = 100


def dijkstra_paths(graph, points, source):
    ds = {source: 0.0}
    prev = {}
    todo = [(0.0, source)]
    while todo:
        cost, u = heapq.heappop(todo)
        if cost > ds[u]:
            continue
        for v in graph[u]:
            nd = cost + math.dist(points[u], points[v])
            if nd < ds.get(v, math.inf):
                ds[v] = nd
                prev[v] = u
                heapq.heappush(todo, (nd, v))
    return ds, prev


def path_of(prev, s, t):
    p = [t]
    while p[-1] != s:
        p.append(prev[p[-1]])
    return p[::-1]


def crossing_windows(pts, regions, radius):
    """Tick windows in which the path crosses each region."""
    segs = []
    cum = 0.0
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        segs.append((a, b, cum, L))
        cum += L
    if not segs:
        segs = [(pts[0], pts[0], 0.0, 0.0)]
    out = []
    for ridx, samples in enumerate(regions):
        d_in = d_out = None
        for a, b, base, L in segs:
            if L <= 1e-9:
                if any(math.hypot(a[0] - sx, a[1] - sy) < radius for sx, sy in samples):
                    d_in = base if d_in is None else min(d_in, base)
                    d_out = base if d_out is None else max(d_out, base)
                continue
            dx, dy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            for sx, sy in samples:
                fx, fy = a[0] - sx, a[1] - sy
                bq = fx * dx + fy * dy
                disc = bq * bq - (fx * fx + fy * fy - radius * radius)
                if disc < 0:
                    continue
                sq = math.sqrt(disc)
                t0, t1 = -bq - sq, -bq + sq
                if t1 < 0 or t0 > L:
                    continue
                e_in = base + max(t0, 0.0)
                e_out = base + min(t1, L)
                d_in = e_in if d_in is None else min(d_in, e_in)
                d_out = e_out if d_out is None else max(d_out, e_out)
        if d_in is not None:
            t_in = int(math.floor(d_in * SCALE + 1e-9))
            t_out = int(math.ceil(d_out * SCALE - 1e-9))
            if t_out <= t_in:
                t_out = t_in + 1
            out.append((ridx, t_in, t_out))
    return out


def region_tables(instance, geo, regions, radius):
    G = len(instance["locations"])
    R = len(instance["robots"])
    cross = {}
    cross_start = {}
    cross_to_start = {}
    inside = {}
    inside_start = {}
    for a in range(G):
        for b in range(G):
            w = crossing_windows(geo["loc_path"][(a, b)], regions, radius)
            if w:
                cross[(a, b)] = w
    for r in range(R):
        for b in range(G):
            w = crossing_windows(geo["start_path"][(r, b)], regions, radius)
            if w:
                cross_start[(r, b)] = w
            w2 = crossing_windows(list(reversed(geo["start_path"][(r, b)])), regions, radius)
            if w2:
                cross_to_start[(b, r)] = w2
    for l, loc in enumerate(instance["locations"]):
        ins = [
            ridx
            for ridx, samples in enumerate(regions)
            if any(math.hypot(loc["position"][0] - sx, loc["position"][1] - sy) < radius for sx, sy in samples)
        ]
        if ins:
            inside[l] = ins
    for r, rb in enumerate(instance["robots"]):
        ins = [
            ridx
            for ridx, samples in enumerate(regions)
            if any(math.hypot(rb["start"][0] - sx, rb["start"][1] - sy) < radius for sx, sy in samples)
        ]
        if ins:
            inside_start[r] = ins
    return dict(
        n=len(regions),
        cross=cross,
        cross_start=cross_start,
        cross_to_start=cross_to_start,
        inside=inside,
        inside_start=inside_start,
    )


def naive_trajectory(instance, schedule, geo):
    """Schedule executed literally on the roadmap: travels depart at the schedule departure times along shortest paths, waits at the destination."""
    G = len(instance["locations"])
    paths = {}
    T = 0.0
    dep0 = schedule.get("start_departure") or {}
    for ri, r in enumerate(instance["robots"]):
        acts = sorted(
            (a for a in schedule["actions"] if a["robot"] == ri), key=lambda a: (a["start"], a["kind"] == "park")
        )
        pc = []
        t = 0.0
        pos = tuple(r["start"])
        cur = ("start", ri)
        prev_dep = dep0.get(str(ri), dep0.get(ri, 0)) / SCALE if acts else 0.0
        if not acts:
            paths[r["name"]] = {"path_cost": [[list(pos), 0.0, [[0.0, 2]]]], "path_time": 0.0}
            continue
        first = True
        for a in acts:
            if a["kind"] == "park":
                dest_key = ("start", a["goal"] - G) if a["goal"] >= G else a["goal"]
                dest = (
                    tuple(instance["robots"][a["goal"] - G]["start"])
                    if a["goal"] >= G
                    else tuple(instance["locations"][a["goal"]]["position"])
                )
            else:
                dest_key = a["goal"]
                dest = tuple(instance["locations"][a["goal"]]["position"])
            dep = max(t, prev_dep)
            wait = dep - t
            if first:
                pc.append([list(pos), wait, [[wait, 2]]] if wait > 1e-9 else tuple(pos))
                first = False
            elif wait > 1e-9:
                if isinstance(pc[-1], list):
                    pc[-1][1] += wait
                    pc[-1][2].append([wait, 2])
                else:
                    pc[-1] = [list(pc[-1]), wait, [[wait, 2]]]
            t = dep
            if cur == dest_key:
                pts = [pos]
            elif isinstance(cur, tuple) and isinstance(dest_key, tuple):
                pts = [pos, dest]
            elif isinstance(cur, tuple):
                pts = geo["start_path"][(cur[1], dest_key)]
            elif isinstance(dest_key, tuple):
                pts = list(reversed(geo["start_path"][(dest_key[1], cur)]))
            else:
                pts = geo["loc_path"][(cur, dest_key)]
            for q in pts[1:-1]:
                t += math.dist(pos, q)
                pc.append(tuple(q))
                pos = q
            if len(pts) > 1:
                t += math.dist(pos, pts[-1])
                pos = tuple(pts[-1])
            cur = dest_key
            if a["kind"] == "park":
                pc.append([list(dest), 0.0, [[0.0, 2]]])
                prev_dep = t
                continue
            parts = []
            w0 = a["start"] / SCALE - t
            if w0 > 1e-9:
                parts.append([w0, 2])
                t += w0
            dur = (a["end"] - a["start"]) / SCALE
            if a["kind"] == "service":
                task = instance["tasks"][a["task"]]
                parts.append([dur, task["type"], instance["goals"][task["goal"]]["name"]])
            else:
                parts.append([dur, 2, a["kind"] + "_tool"])
            t += dur
            pc.append([list(dest), sum(p[0] for p in parts), parts])
            prev_dep = max(t, a.get("dep", a["end"]) / SCALE)
        paths[r["name"]] = {"path_cost": pc, "path_time": t}
        T = max(T, t)
    return paths, T


def worker(args):
    t0 = args.start
    out = Path(args.output).resolve()
    deadline = t0 + args.budget
    os.chdir(out)
    os.environ["MPLBACKEND"] = "Agg"
    if args.cores:
        os.sched_setaffinity(0, {int(x) for x in args.cores.split(",")})
    stack = Path(args.stack_root).resolve()
    sys.path.insert(0, str(stack))
    sys.path.insert(0, str(stack / "code"))
    import numpy as np
    from world.environment import Environment
    from world.prm import PRMPlanning
    from task_planning.pddl_vrp_generator import Planner
    from motion.prioritized import PrioritizedPlanningSolver
    from motion.final_validator import validate_final, _timed_events
    from motion.collision_vor import collision_check
    import baseline_tool_adapter as bta
    import tool_adapter
    from joint_model import solve

    random.seed(args.prm_seed)
    np.random.seed(args.prm_seed)
    ev = open(out / "events.jsonl", "a")

    def event(**kw):
        ev.write(json.dumps(dict(t=round(time.monotonic() - t0, 3), **kw), default=enc) + "\n")
        ev.flush()

    def check(stage):
        if time.monotonic() >= deadline:
            raise TimeoutError(stage)

    tool = args.mission == "tool"
    modules = {}
    ts = time.monotonic()
    env = Environment(args.input, False)
    prm = PRMPlanning(env)
    samples, graph, goals = prm.ConstructPhase(goals_map=env.goals_map)
    initial = dict(samples=samples, roadmap=graph, goals=goals)
    prm.smooth = True
    world = SimpleNamespace(env=env, prm=prm)
    modules["prm"] = time.monotonic() - ts
    save(
        out / "roadmap.json",
        dict(prm_seed=args.prm_seed, construction_s=modules["prm"], graph=initial),
    )
    generator = Planner(env, prm, "tamp", "unused.pddl")
    generator.env_instance()
    radii = {r.robot_radius for r in env.robots_map.values()}
    if len(radii) != 1:
        raise ValueError("COMMON_RADIUS_REQUIRED")
    rr = next(iter(radii))
    region_radius = prm.max_rr * 2
    instance = build_instance(generator, world, Path(args.input).stem, tool)
    save(out / "instance.json", instance)
    pts = [tuple(x) for x in prm.task_samples]
    index = {p: i for i, p in enumerate(pts)}
    G = len(instance["locations"])
    targets = [index[tuple(l["position"])] for l in instance["locations"]]
    starts = [index[tuple(r["start"])] for r in instance["robots"]]
    cache = {s: dijkstra_paths(prm.task_roadmap, pts, s) for s in set(targets + starts)}
    geo = {"points": pts, "loc_path": {}, "start_path": {}}
    for a in range(G):
        for b in range(G):
            p = path_of(cache[targets[a]][1], targets[a], targets[b]) if a != b else [targets[a]]
            geo["loc_path"][(a, b)] = [pts[i] for i in p]
    for r in range(len(instance["robots"])):
        for b in range(G):
            p = path_of(cache[starts[r]][1], starts[r], targets[b]) if starts[r] != targets[b] else [starts[r]]
            geo["start_path"][(r, b)] = [pts[i] for i in p]
    common = dict(
        method="JOINT_FB",
        case=Path(args.input).stem,
        prm_seed=args.prm_seed,
        cp_seed=args.cp_seed,
        policy=f"gap{args.gap}_cap{args.cp_max_s}s_escalate + collision feedback (regions as spatial resources on the occupancy model), one SIPP per round, stop at first valid plan",
    )
    candidates = []
    incumbent = None
    col_env = []
    tmem = []
    rounds = []
    count = 0
    seen = {}
    first_round = None
    last_failure = None
    (out / "candidates").mkdir(exist_ok=True)

    def validate(paths):
        v = validate_final(paths, generator.obj_ins, env.robots_map)
        if tool:
            v = v + [
                (e, "portable tool")
                for e in tool_adapter.validate_tools(SimpleNamespace(tool=True, obj_ins=generator.obj_ins), paths)
            ]
        horizon = max((e[-1][1] for p in paths.values() if (e := _timed_events(p["path_cost"])[0])), default=0.0)
        return v, horizon

    def finish(stop_reason, end_reason=None, **kw):
        now = time.monotonic() - t0
        ev.flush()
        save(out / "rounds.json", rounds)
        base = dict(
            **common,
            runtime_s=now,
            rounds=len(rounds),
            candidates=len(candidates),
            regions=len(col_env),
            transient=len(tmem),
            modules=modules,
            first_round=first_round,
            stop_reason=stop_reason,
            last_failure=last_failure,
            **kw,
        )
        if incumbent is not None:
            save(
                out / "result.json",
                dict(
                    base,
                    solved=True,
                    makespan=incumbent["trajectory_makespan"],
                    schedule_makespan=incumbent.get("schedule_makespan"),
                    end_reason="VALID",
                    validation_complete_at_s=incumbent["validated_elapsed_s"],
                    incumbent_candidate=incumbent["candidate_id"],
                    incumbent_round=incumbent["round"],
                ),
            )
        else:
            save(
                out / "result.json",
                dict(
                    base,
                    solved=False,
                    makespan=None,
                    end_reason=end_reason or (last_failure or {}).get("class") or stop_reason,
                ),
            )

    try:
        while True:
            check("scheduler")
            regions = [list(e[1]) for e in col_env if e[1]] + [list(m["samples"]) for m in tmem if m["samples"]]
            tables = (
                region_tables(instance, geo, [[tuple(s) for s in rg] for rg in regions], region_radius)
                if regions
                else None
            )
            save(
                out / f"regions_{count:02d}.json",
                dict(
                    round=count,
                    structural=[list(e[1]) for e in col_env if e[1]],
                    transient=[list(m["samples"]) for m in tmem],
                    n=len(regions),
                    tables=(
                        None
                        if tables is None
                        else dict(
                            n=tables["n"],
                            cross=len(tables["cross"]),
                            cross_start=len(tables["cross_start"]),
                            cross_to_start=len(tables["cross_to_start"]),
                            inside=tables["inside"],
                            inside_start=tables["inside_start"],
                        )
                    ),
                ),
            )
            cap = args.cp_max_s
            attempt = 0
            schedule = None
            cp_log = []
            while True:
                check("scheduler")
                ts = time.monotonic()
                schedule = solve(
                    instance,
                    seconds=cap,
                    workers=4,
                    seed=args.cp_seed,
                    wall_deadline=deadline,
                    gap_limit=args.gap,
                    cp_max_s=cap,
                    occupancy=True,
                    regions=tables,
                )
                dt = time.monotonic() - ts
                modules["scheduler"] = modules.get("scheduler", 0) + dt
                cp_log.append(
                    dict(
                        attempt=attempt,
                        cap_s=cap,
                        status=schedule["status"],
                        solver_s=schedule.get("solver_seconds"),
                        makespan=schedule.get("makespan"),
                        first_feasible_s=(
                            (schedule.get("incumbent_updates") or [{}])[0].get("solver_elapsed_s")
                            if schedule.get("incumbent_updates")
                            else None
                        ),
                    )
                )
                event(
                    type="cpsat",
                    round=count,
                    **cp_log[-1],
                    regions=len(regions),
                    region_intervals=schedule.get("region_intervals"),
                )
                if schedule.get("validated") or schedule.get("validation_errors") or args.no_escalation:
                    break
                if time.monotonic() + cap * 2 > deadline:
                    break
                cap *= 2
                attempt += 1
            save(out / f"schedule_{count:02d}.json", dict(schedule, cp_log=cp_log))
            rd = dict(
                round=count,
                cp_status=schedule["status"],
                cp_attempts=cp_log,
                schedule_makespan=schedule.get("makespan"),
                lower_bound=schedule.get("lower_bound"),
                regions=len(regions),
                region_intervals=schedule.get("region_intervals"),
                elapsed_s=round(time.monotonic() - t0, 3),
            )
            if not schedule.get("validated"):
                cls = "SCHEDULE_INVALID" if schedule.get("validation_errors") else "NO_SCHEDULE"
                last_failure = dict(round=count, **{"class": cls}, cp_status=schedule["status"], cp_attempts=cp_log)
                rounds.append(dict(rd, outcome=cls))
                if count == 0:
                    first_round = dict(
                        success=False,
                        stage=cls,
                        schedule_within_cap=False,
                        cp_attempts=cp_log,
                        elapsed_s=rd["elapsed_s"],
                    )
                return finish(cls)
            h = sha({"actions": schedule["actions"]})
            rep = seen.get(h)
            seen.setdefault(h, count)
            rd["repeated_from"] = rep
            # SIPP (one attempt) + validation
            check("sipp")
            plan, events = export_sipp(instance, schedule)
            save(out / f"sipp_input_{count:02d}.json", dict(plan=plan, tool_events=events))
            ts = time.monotonic()
            st = {n: r.pos for n, r in env.robots_map.items()}
            solver = None
            paths = None
            sipp_err = None
            try:
                solver = (
                    bta.ToolPrioritizedSolver(
                        prm.task_roadmap,
                        prm.task_samples,
                        plan,
                        events,
                        rr=rr,
                        starts=st,
                        budget_check=lambda: time.monotonic() >= deadline,
                    )
                    if tool
                    else PrioritizedPlanningSolver(prm.task_roadmap, prm.task_samples, plan, rr=rr, starts=st)
                )
                paths, reported, total = solver.find_solution()
            except Exception as e:
                sipp_err = repr(e)[:200]
            modules["sipp"] = modules.get("sipp", 0) + time.monotonic() - ts
            save(
                out / f"sipp_{count:02d}.json",
                dict(
                    ok=bool(paths),
                    robot_path=paths,
                    handover_failure=getattr(solver, "handover_failure", None),
                    error=sipp_err,
                ),
            )
            rec = None
            if paths:
                check("validation")
                v, horizon = validate(paths)
                now = time.monotonic() - t0
                rec = dict(
                    candidate_id=len(candidates),
                    round=count,
                    source="sipp",
                    valid=not v,
                    violations=v,
                    trajectory_makespan=horizon,
                    schedule_makespan=schedule["makespan"],
                    validated_elapsed_s=now,
                )
                candidates.append(rec)
                save(out / f'candidates/{rec["candidate_id"]:04d}.json', dict(rec, robot_path=paths))
                event(
                    type="candidate",
                    round=count,
                    valid=rec["valid"],
                    makespan=horizon,
                    violation_classes=sorted({k for k, _ in v}),
                )
                rd.update(
                    sipp="ok",
                    validation="valid" if rec["valid"] else "invalid",
                    violation_classes=sorted({k for k, _ in v}),
                    trajectory_makespan=horizon,
                )
                if rec["valid"] and now <= args.budget:
                    incumbent = rec
                elif not rec["valid"]:
                    last_failure = dict(
                        round=count, **{"class": "VALIDATION_FAILED"}, violation_classes=rd["violation_classes"]
                    )
            else:
                hf = getattr(solver, "handover_failure", None)
                cause = (hf.get("cause") if isinstance(hf, dict) else hf) or sipp_err or "no path"
                rd.update(sipp="failed", sipp_cause=str(cause)[:120], validation=None)
                last_failure = dict(round=count, **{"class": "SIPP_FAILED"}, cause=str(cause)[:120])
                event(type="sipp", round=count, ok=False, cause=str(cause)[:120])
            if count == 0:
                first_round = dict(
                    success=incumbent is not None,
                    makespan=incumbent["trajectory_makespan"] if incumbent else None,
                    schedule_makespan=schedule["makespan"],
                    schedule_within_cap=(cp_log[0]["status"] in ("FEASIBLE", "OPTIMAL")),
                    cp_attempts=cp_log,
                    sipp=rd.get("sipp"),
                    validation=rd.get("validation"),
                    violation_classes=rd.get("violation_classes"),
                    elapsed_s=round(time.monotonic() - t0, 3),
                )
            if incumbent is not None:
                rounds.append(dict(rd, outcome="VALID"))
                return finish("VALID_PLAN")
            # feedback: replay the schedule, extract collision regions
            check("feedback")
            ts = time.monotonic()
            naive, T_naive = naive_trajectory(instance, schedule, geo)
            save(out / f"naive_{count:02d}.json", dict(robot_path=naive, makespan=T_naive))
            col, env_v = collision_check(naive, env.robots_map, prm, col_env, tmem, count)
            modules["collision"] = modules.get("collision", 0) + time.monotonic() - ts
            new_regions = (len(env_v) if isinstance(env_v, list) else 0) - len(col_env)
            rd.update(naive_makespan=T_naive, collisions=len(col), new_regions=new_regions, transient=len(tmem))
            event(
                type="feedback",
                round=count,
                collisions=len(col),
                new_regions=new_regions,
                transient=len(tmem),
                last_failure=last_failure,
            )
            if not col:
                rounds.append(dict(rd, outcome="FEEDBACK_UNAVAILABLE"))
                return finish("FEEDBACK_UNAVAILABLE")
            if rep is not None and new_regions == 0 and not tmem:
                rounds.append(dict(rd, outcome="FEEDBACK_INEFFECTIVE"))
                return finish("FEEDBACK_INEFFECTIVE")
            if isinstance(env_v, list):
                col_env = env_v
            rounds.append(dict(rd, outcome="FEEDBACK"))
            count += 1
    except TimeoutError as e:
        rounds.append(dict(round=count, outcome="BUDGET", stage=str(e)))
        finish("BUDGET_EXHAUSTED", end_reason="BUDGET_EXHAUSTED", budget_stage=str(e))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--stack-root", type=Path, default=ROOT / "vendor")
    p.add_argument("--mission", choices=["precedence", "tool"], required=True)
    p.add_argument("--prm-seed", type=int, default=0)
    p.add_argument("--cp-seed", type=int, default=0)
    p.add_argument("--budget", type=float, default=100)
    p.add_argument("--cores", default="")
    p.add_argument("--gap", type=float, default=0.05)
    p.add_argument("--cp-max-s", type=float, default=5.0)
    p.add_argument(
        "--no-escalation",
        action="store_true",
        help="strict per-call cap (no doubling when no feasible schedule appears)",
    )
    p.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--start", type=float, default=0, help=argparse.SUPPRESS)
    args = p.parse_args()
    args.input = args.input.resolve()
    args.output = args.output.resolve()
    args.stack_root = args.stack_root.resolve()
    if args.worker:
        try:
            worker(args)
        except Exception as e:
            traceback.print_exc()
            save(
                args.output / "error.json",
                dict(error=repr(e), traceback=traceback.format_exc(), runtime_s=time.monotonic() - args.start),
            )
            if not (args.output / "result.json").exists():
                save(
                    args.output / "result.json",
                    dict(
                        solved=False,
                        makespan=None,
                        end_reason="ERROR",
                        error=repr(e),
                        runtime_s=time.monotonic() - args.start,
                    ),
                )
        return
    if args.output.exists() and any(args.output.iterdir()):
        raise SystemExit("Output directory must be empty; no result overwrites.")
    args.output.mkdir(parents=True, exist_ok=True)
    cores = args.cores or ",".join(str(x) for x in sorted(os.sched_getaffinity(0))[:4])
    if len(set(cores.split(","))) != 4:
        raise SystemExit("Exactly four cores required.")
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--input",
        str(args.input),
        "--output",
        str(args.output),
        "--stack-root",
        str(args.stack_root),
        "--mission",
        args.mission,
        "--budget",
        str(args.budget),
        "--prm-seed",
        str(args.prm_seed),
        "--cp-seed",
        str(args.cp_seed),
        "--cores",
        cores,
        "--gap",
        str(args.gap),
        "--cp-max-s",
        str(args.cp_max_s),
    ] + (["--no-escalation"] if args.no_escalation else [])
    start = time.monotonic()
    command += ["--start", str(start)]
    with (args.output / "stdout.txt").open("w") as so, (args.output / "stderr.txt").open("w") as se:
        proc = subprocess.Popen(command, stdout=so, stderr=se, start_new_session=True)
        timeout = False
        try:
            proc.wait(timeout=max(0.001, args.budget + 5 - (time.monotonic() - start)))
        except subprocess.TimeoutExpired:
            timeout = True
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    elapsed = time.monotonic() - start
    result = args.output / "result.json"
    if not result.exists():
        save(
            result,
            dict(
                method="JOINT_FB",
                solved=False,
                makespan=None,
                end_reason="TOTAL_TIMEOUT" if timeout else "WORKER_CRASH",
                runtime_s=min(elapsed, args.budget),
                returncode=proc.returncode,
            ),
        )
    data = json.loads(result.read_text())
    if data.get("solved") and data.get("validation_complete_at_s", math.inf) > args.budget:
        data.update(solved=False, makespan=None, end_reason="VALIDATION_OVER_BUDGET")
        save(result, data)
    save(
        args.output / "supervisor.json",
        dict(timeout=timeout, elapsed_including_cleanup_s=elapsed, returncode=proc.returncode, command=command),
    )
    print(
        json.dumps(
            {
                k: data.get(k)
                for k in ("case", "solved", "makespan", "end_reason", "stop_reason", "rounds", "first_round")
            },
            default=enc,
        )
    )


if __name__ == "__main__":
    main()
