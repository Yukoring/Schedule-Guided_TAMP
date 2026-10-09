#!/usr/bin/env python3
"""TAS+SIPP: joint CP-SAT schedule -> one prioritized SIPP attempt -> validation."""
import argparse
import hashlib
import heapq
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import time
import traceback
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent


def enc(x):
    if hasattr(x, "tolist"):
        return x.tolist()
    if hasattr(x, "item"):
        return x.item()
    if isinstance(x, set):
        return sorted(x)
    raise TypeError(type(x).__name__)


def save(path, obj):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, default=enc) + "\n")
    tmp.replace(path)


def sha(x):
    return hashlib.sha256(json.dumps(x, sort_keys=True, default=enc).encode()).hexdigest()


def dijkstra(graph, points, source):
    ds = {source: 0.0}
    todo = [(0.0, source)]
    while todo:
        cost, u = heapq.heappop(todo)
        if cost > ds[u]:
            continue
        for v in graph[u]:
            nd = cost + math.dist(points[u], points[v])
            if nd < ds.get(v, math.inf):
                ds[v] = nd
                heapq.heappush(todo, (nd, v))
    return ds


def build_instance(generator, world, name, tool):
    from joint_model import ticks

    obj = generator.obj_ins
    ways = {w.ind: w for w in obj["waypoint"]}
    jobs = obj["job"]
    locids = list(dict.fromkeys(w for j in jobs for w in j.located))
    locindex = {w: i for i, w in enumerate(locids)}
    locations = [dict(name=w, position=list(ways[w].loc)) for w in locids]
    robots = [dict(name=r.ind, start=list(ways[r.at[0]].loc), durations=dict(r.action_duration)) for r in obj["robot"]]
    tasks, lookup = [], {}
    for g, j in enumerate(jobs):
        for t in obj["task"]:
            if j.ind in t.loc:
                lookup[j.ind, t.ind] = len(tasks)
                tasks.append(
                    dict(
                        name=f"{j.ind}:{t.ind}",
                        goal=g,
                        type=t.ind,
                        locations=[locindex[w] for w in j.located],
                        tool=tool and t.ind in ("taska", "taskb"),
                    )
                )
    precedence = [[lookup[j.ind, a], lookup[j.ind, b]] for j in jobs for a, b in j.order]
    samples = [tuple(x) for x in world.prm.task_samples]
    graph = world.prm.task_roadmap
    index = {p: i for i, p in enumerate(samples)}
    targets = [index[tuple(l["position"])] for l in locations]
    starts = [index[tuple(r["start"])] for r in robots]
    cache = {s: dijkstra(graph, samples, s) for s in set(targets + starts)}

    def distance(s, t):
        # Never substitute a Euclidean shortcut for a missing PRM connection.
        if t not in cache[s]:
            raise ValueError("DISCONNECTED_SHARED_PRM")
        return ticks(cache[s][t])

    first = next(j for j in jobs if j.ind == "goal0000")
    return dict(
        name=name,
        robots=robots,
        goals=[dict(name=j.ind, position=list(ways[j.located[0]].loc)) for j in jobs],
        locations=locations,
        tasks=tasks,
        precedence=precedence,
        speed=1.0,
        tool_initial_goal=next(i for i, j in enumerate(jobs) if j.ind == "goal0000"),
        tool_initial_location=locindex[first.located[0]],
        handling_seconds=0.5,
        handover_gap_ticks=1,
        travel_ticks=[[distance(a, b) for b in targets] for a in targets],
        start_travel_ticks=[[distance(a, b) for b in targets] for a in starts],
    )


def export_sipp(instance, schedule):
    """Turn the schedule into the SIPP plan layout; assignment, order and locations are kept, SIPP only retimes."""
    plans = {r["name"]: [] for r in instance["robots"]}
    events = []
    for ri, r in enumerate(instance["robots"]):
        pos = tuple(r["start"])
        for a in sorted((a for a in schedule["actions"] if a["robot"] == ri), key=lambda a: a["start"]):
            if a["kind"] == "park":
                G_ = len(instance["locations"])
                dest = (
                    tuple(instance["robots"][a["goal"] - G_]["start"])
                    if a["goal"] >= G_
                    else tuple(instance["locations"][a["goal"]]["position"])
                )
                if pos != dest:
                    plans[r["name"]].append([pos, dest])
                pos = dest
                continue
            dest = tuple(instance["locations"][a["goal"]]["position"])
            # A zero-length first navigation also gives the native parser its start.
            if pos != dest or not plans[r["name"]]:
                plans[r["name"]].append([pos, dest])
            if a["kind"] == "service":
                task = instance["tasks"][a["task"]]
                plans[r["name"]].append(
                    [dest, (a["end"] - a["start"]) / 100, task["type"], instance["goals"][task["goal"]]["name"]]
                )
            else:
                kind = a["kind"] + "_tool"
                plans[r["name"]].append([dest, (a["end"] - a["start"]) / 100, 2, kind])
                events.append(
                    dict(
                        t=a["start"] / 100, robot=r["name"], kind=kind, loc=dest, robot_index=len(plans[r["name"]]) - 1
                    )
                )
            pos = dest
    events.sort(key=lambda e: (e["t"], 0 if e["kind"] == "drop_tool" else 1))
    return plans, events


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
    import baseline_tool_adapter as bta
    import tool_adapter
    from joint_model import solve

    random.seed(args.prm_seed)
    np.random.seed(args.prm_seed)
    modules = {}

    def check():
        if time.monotonic() >= deadline:
            raise TimeoutError("TOTAL_BUDGET")

    def stage(name):
        check()
        save(out / "stage.json", dict(stage=name, elapsed_s=time.monotonic() - t0))

    tool = args.mission == "tool"
    stage("prm")
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
    stage("model_setup")
    ts = time.monotonic()
    generator = Planner(env, prm, "tamp", "unused.pddl")
    generator.env_instance()
    radii = {r.robot_radius for r in env.robots_map.values()}
    if len(radii) != 1:
        raise ValueError("COMMON_RADIUS_REQUIRED")
    instance = build_instance(generator, world, Path(args.input).stem, tool)
    save(out / "instance.json", instance)
    modules["model_setup"] = time.monotonic() - ts
    stage("scheduler")
    ts = time.monotonic()
    schedule = (
        solve(
            instance,
            seconds=5,
            workers=4,
            seed=args.cp_seed,
            wall_deadline=deadline,
            gap_limit=args.gap,
            cp_max_s=args.cp_max_s,
            occupancy=args.occupancy,
        )
        if args.gap is not None
        else solve(
            instance,
            seconds=5,
            workers=4,
            seed=args.cp_seed,
            wall_deadline=deadline,
            first_after_base=True,
            occupancy=args.occupancy,
        )
    )
    modules["scheduler"] = time.monotonic() - ts
    save(out / "schedule.json", schedule)
    check()
    common = dict(
        method="JOINT_TOOL_SIPP",
        case=Path(args.input).stem,
        modules=modules,
        prm_seed=args.prm_seed,
        cp_seed=args.cp_seed,
    )

    def finish(reason, **kw):
        save(
            out / "result.json",
            dict(**common, solved=False, makespan=None, end_reason=reason, runtime_s=time.monotonic() - t0, **kw),
        )

    if not schedule.get("validated"):
        return finish(
            "SCHEDULE_INVALID" if schedule.get("validation_errors") else "NO_SCHEDULE", solver_status=schedule["status"]
        )
    plan, events = export_sipp(instance, schedule)
    save(out / "sipp_input.json", dict(plan=plan, tool_events=events))
    stage("sipp")
    ts = time.monotonic()
    starts = {n: r.pos for n, r in env.robots_map.items()}
    if tool:
        solver = bta.ToolPrioritizedSolver(
            prm.task_roadmap,
            prm.task_samples,
            plan,
            events,
            rr=next(iter(radii)),
            starts=starts,
            budget_check=lambda: time.monotonic() >= deadline,
        )
    else:
        solver = PrioritizedPlanningSolver(
            prm.task_roadmap, prm.task_samples, plan, rr=next(iter(radii)), starts=starts
        )
    paths, reported_makespan, total_cost = solver.find_solution()
    modules["sipp"] = time.monotonic() - ts
    save(
        out / "motion.json",
        dict(
            robot_path=paths,
            reported_makespan=reported_makespan,
            total_cost=total_cost,
            handover_failure=getattr(solver, "handover_failure", None),
            handover_history=getattr(solver, "handover_history", None),
        ),
    )
    check()
    if not paths:
        return finish("SIPP_FAILED")
    stage("validation")
    ts = time.monotonic()
    violations = validate_final(paths, generator.obj_ins, env.robots_map)
    if tool:
        violations += [
            (e, "portable tool")
            for e in tool_adapter.validate_tools(SimpleNamespace(tool=True, obj_ins=generator.obj_ins), paths)
        ]
    horizon = max((ev[-1][1] for p in paths.values() if (ev := _timed_events(p["path_cost"])[0])), default=0.0)
    completed = time.monotonic() - t0
    modules["validation"] = time.monotonic() - ts
    check()
    save(out / "validation.json", dict(violations=violations, complete_at_s=completed, trajectory_makespan=horizon))
    save(
        out / "result.json",
        dict(
            **common,
            solved=not violations,
            makespan=horizon if not violations else None,
            schedule_makespan=schedule["makespan"],
            end_reason="VALID" if not violations else "VALIDATION_FAILED",
            runtime_s=completed,
            validation_complete_at_s=completed,
            violations=violations,
        ),
    )


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
    p.add_argument("--gap", type=float, default=None, help="relative gap limit for early CP-SAT stop (gap policy)")
    p.add_argument("--cp-max-s", type=float, default=None, help="CP-SAT time cap under the gap policy")
    p.add_argument(
        "--occupancy",
        action="store_true",
        help="vertex occupancy + explicit parking in the joint model (no-park configuration)",
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
                args.output / "result.json",
                dict(
                    solved=False,
                    makespan=None,
                    end_reason="TOTAL_TIMEOUT" if isinstance(e, TimeoutError) else "ERROR",
                    error=repr(e),
                    traceback=traceback.format_exc(),
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
    if not {int(x) for x in cores.split(",")}.issubset(os.sched_getaffinity(0)):
        raise SystemExit("Requested cores are outside the available CPU affinity.")
    command = (
        [
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
        ]
        + (["--gap", str(args.gap), "--cp-max-s", str(args.cp_max_s)] if args.gap is not None else [])
        + (["--occupancy"] if args.occupancy else [])
    )
    start = time.monotonic()
    command += ["--start", str(start)]
    with (args.output / "stdout.txt").open("w") as stdout, (args.output / "stderr.txt").open("w") as stderr:
        proc = subprocess.Popen(command, stdout=stdout, stderr=stderr, start_new_session=True)
        timeout = False
        try:
            proc.wait(timeout=max(0.001, args.budget - (time.monotonic() - start)))
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
    print(json.dumps(data, default=enc))


if __name__ == "__main__":
    main()
