#!/usr/bin/env python3
"""Joint CP-SAT model for task allocation and scheduling with a portable tool: assignment, order, timing, precedence, tool pickup/drop/hand-over, vertex occupancy and parking. Times in 0.01 s ticks."""
import argparse
import copy
import json
import math
import time
import threading
from pathlib import Path

import yaml
from ortools.sat.python import cp_model

SCALE = 100


def ticks(seconds):
    return int(math.ceil(seconds * SCALE - 1e-8))


def load_yaml(path):
    raw = yaml.safe_load(Path(path).read_text())
    if raw["environment"].get("obstacles"):
        raise ValueError("This prototype only supports obstacle-free inputs.")
    robots = []
    for name, data in raw["robots"]["robot"].items():
        types = [s.strip() for s in data["service"].split(",")]
        robots.append(dict(name=name, start=data["state"][:2], durations=dict(zip(types, data["action_cost"]))))
    goals, tasks, precedence = [], [], []
    for name, data in raw["goals"]["goal"].items():
        pts = data["corners"]
        center = [(min(p[k] for p in pts) + max(p[k] for p in pts)) / 2 for k in (0, 1)]
        g = len(goals)
        goals.append(dict(name=name, position=center))
        by_type = {}
        for typ in (s.strip() for s in data["service"].split(",")):
            by_type[typ] = len(tasks)
            tasks.append(dict(name=f"{name}:{typ}", goal=g, type=typ, tool=typ in ["taska", "taskb"]))
        for relation in data.get("order", []):
            a, b = relation.split("-")
            precedence.append([by_type[a], by_type[b]])
    # tool semantics follow the benchmark convention
    return dict(
        name=Path(path).stem,
        robots=robots,
        goals=goals,
        tasks=tasks,
        tool_initial_goal=0,
        handling_seconds=0.5,
        speed=1.0,
        precedence=precedence,
    )


def solve(
    instance,
    seconds=5.0,
    workers=4,
    seed=0,
    deadline=None,
    wall_deadline=None,
    first_after_base=False,
    gap_limit=None,
    cp_max_s=None,
    occupancy=False,
    regions=None,
):
    """Vertex occupancy and explicit parking: a node occupies its location from arrival to departure, the start vertex until the first departure, and each robot parks at a chosen vertex until the end of the plan."""
    """Collision regions as shared resources: a travel crossing a region occupies it for its crossing window, nodes and parking inside a region occupy it; NoOverlap per region."""
    """Stop at the relative gap or at the time cap; the best schedule found is returned."""
    build_start = time.perf_counter()
    m = cp_model.CpModel()
    robots, tasks, goals = (instance[k] for k in ["robots", "tasks", "goals"])
    R = len(robots)
    locations = instance.get("locations", goals)
    G = len(locations)
    handle = ticks(instance["handling_seconds"])
    speed = instance["speed"]
    points = [g["position"] for g in locations]
    dist = [[ticks(math.dist(a, b) / speed) for b in points] for a in points]
    startdist = [[ticks(math.dist(r["start"], b) / speed) for b in points] for r in robots]
    if "travel_ticks" in instance:
        dist = instance["travel_ticks"]
        startdist = instance["start_travel_ticks"]
    # horizon bound
    H = max(
        10000,
        sum(ticks(max(r["durations"].get(t["type"], 0) for r in robots)) for t in tasks)
        + len(tasks) * 10 * (max(map(max, dist)) + handle + 1)
        + max(map(max, startdist)),
    )
    nodes, service, pickup, drop, owners = [], {}, {}, {}, []

    def node(kind, j, loc, present):
        idx = len(nodes)
        s = m.new_int_var(0, H, f"{kind}{j}_start")
        e = m.new_int_var(0, H, f"{kind}{j}_end")
        if kind != "service":
            m.add(e == s + handle).only_enforce_if(present)
            m.add(s == 0).only_enforce_if(present.Not())
            m.add(e == 0).only_enforce_if(present.Not())
        nodes.append(dict(kind=kind, task=j, loc=loc, present=present, s=s, e=e))
        return idx

    for j, t in enumerate(tasks):
        capable = [r for r in range(R) if t["type"] in robots[r]["durations"]]
        if not capable:
            raise ValueError(f"No capable robot for task {j}")
        owner = m.new_int_var_from_domain(cp_model.Domain.from_values(capable), f"owner{j}")
        owners.append(owner)
        options = t.get("locations", [t["goal"]])
        loc = (
            options[0]
            if len(options) == 1
            else m.new_int_var_from_domain(cp_model.Domain.from_values(options), f"service_location{j}")
        )
        n = node("service", j, loc, m.new_constant(1))
        service[j] = n
        durations = [ticks(r["durations"].get(t["type"], 0)) for r in robots]
        d = m.new_int_var(min(durations[r] for r in capable), max(durations), f"duration{j}")
        m.add_element(owner, durations, d)
        m.add(nodes[n]["e"] == nodes[n]["s"] + d)
    tooljobs = [j for j, t in enumerate(tasks) if t["tool"]]
    for j in tooljobs:
        ploc = m.new_int_var(0, G - 1, f"pickup{j}_location")
        pickup[j] = node("pickup", j, ploc, m.new_bool_var(f"pickup{j}_present"))
        drop[j] = node("drop", j, nodes[service[j]]["loc"], m.new_bool_var(f"drop{j}_present"))
        m.add(nodes[service[j]]["s"] >= nodes[pickup[j]]["e"]).only_enforce_if(nodes[pickup[j]]["present"])
        m.add(nodes[drop[j]]["s"] >= nodes[service[j]]["e"]).only_enforce_if(nodes[drop[j]]["present"])

    toolarcs, tool_selected = [], {}
    incoming = {j: [] for j in tooljobs}
    outgoing = {j: [] for j in tooljobs}
    ids = {j: k + 1 for k, j in enumerate(tooljobs)}
    for j in tooljobs:
        first = m.new_bool_var(f"tool_first{j}")
        last = m.new_bool_var(f"tool_last{j}")
        toolarcs.extend([(0, ids[j], first), (ids[j], 0, last)])
        tool_selected[-1, j] = first
        tool_selected[j, -1] = last
        incoming[j].append(first)
        m.add(
            nodes[pickup[j]]["loc"] == instance.get("tool_initial_location", instance["tool_initial_goal"])
        ).only_enforce_if(first)
    for i in tooljobs:
        for j in tooljobs:
            if i == j:
                continue
            arc = m.new_bool_var(f"tool_arc{i}_{j}")
            same = m.new_bool_var(f"same_owner{i}_{j}")
            m.add(owners[i] == owners[j]).only_enforce_if(same)
            m.add(owners[i] != owners[j]).only_enforce_if(same.Not())
            transfer = m.new_bool_var(f"transfer{i}_{j}")
            m.add(transfer <= arc)
            m.add(transfer + same <= 1)
            m.add(transfer >= arc - same)
            incoming[j].append(transfer)
            outgoing[i].append(transfer)
            toolarcs.append((ids[i], ids[j], arc))
            tool_selected[i, j] = arc
            m.add(nodes[service[j]]["s"] >= nodes[service[i]]["e"]).only_enforce_if(arc)
            m.add(nodes[pickup[j]]["loc"] == nodes[service[i]]["loc"]).only_enforce_if(arc)
            m.add(nodes[pickup[j]]["s"] >= nodes[drop[i]]["e"] + instance.get("handover_gap_ticks", 0)).only_enforce_if(
                transfer
            )
    if tooljobs:
        m.add_circuit(toolarcs)
    for j in tooljobs:
        m.add(nodes[pickup[j]]["present"] == sum(incoming[j]))
        m.add(nodes[drop[j]]["present"] == sum(outgoing[j]))

    # Shared travel expressions: pickup locations are decision variables.
    travel_cache = {}

    def travel(a, b):
        key = (a, b)
        if key in travel_cache:
            return travel_cache[key]
        la, lb = nodes[a]["loc"], nodes[b]["loc"]
        if isinstance(la, int) and isinstance(lb, int):
            out = dist[la][lb]
        else:
            ix = m.new_int_var(0, G * G - 1, f"index{a}_{b}")
            out = m.new_int_var(0, max(map(max, dist)), f"travel{a}_{b}")
            m.add(ix == la * G + lb)
            m.add_element(ix, [v for row in dist for v in row], out)
        travel_cache[key] = out
        return out

    robot_arc_count = 0
    occ = {}
    park = {}
    region_ivs = {i: [] for i in range(regions["n"])} if regions else {}

    def both(a, b, name):
        lit = m.new_bool_var(name)
        m.add_bool_and([a, b]).only_enforce_if(lit)
        m.add_bool_or([a.Not(), b.Not()]).only_enforce_if(lit.Not())
        return lit

    def add_crossings(dep, lit, la, lb, name):
        """Region occupancy of a travel from la to lb departing at dep."""
        if not regions:
            return
        if isinstance(la, tuple):
            table, keyf, idx, keys = (
                regions["cross_start"],
                None,
                (lb if not isinstance(lb, int) else None),
                [(la[1], b) for b in range(G)],
            )
        elif isinstance(lb, tuple):
            table, idx, keys = (
                regions["cross_to_start"],
                (la if not isinstance(la, int) else None),
                [(a, lb[1]) for a in range(G)],
            )
        else:
            table = regions["cross"]
            idx = None
            keys = None
        if (
            isinstance(la, tuple)
            and isinstance(lb, int)
            or isinstance(lb, tuple)
            and isinstance(la, int)
            or (isinstance(la, int) and isinstance(lb, int))
        ):
            key = (la[1], lb) if isinstance(la, tuple) else ((la, lb[1]) if isinstance(lb, tuple) else (la, lb))
            for ridx, t_in, t_out in table.get(key, []):
                sv = m.new_int_var(0, 2 * H, "")
                m.add(sv == dep + t_in)
                ev_ = m.new_int_var(0, 2 * H, "")
                m.add(ev_ == dep + t_out)
                region_ivs[ridx].append(m.new_optional_interval_var(sv, t_out - t_in, ev_, lit, f"{name}_occ{ridx}"))
            return
        if keys is None:
            if isinstance(lb, int):
                idx = la
                keys = [(a, lb) for a in range(G)]
            elif isinstance(la, int):
                idx = lb
                keys = [(la, b) for b in range(G)]
            else:
                idx = m.new_int_var(0, G * G - 1, f"{name}_pix")
                m.add(idx == la * G + lb)
                keys = [(a, b) for a in range(G) for b in range(G)]
        for ridx in range(regions["n"]):
            tin = []
            tout = []
            cross = []
            for k in keys:
                hit = next(((ti, to) for rr, ti, to in table.get(k, []) if rr == ridx), None)
                cross.append(1 if hit else 0)
                tin.append(hit[0] if hit else 0)
                tout.append(hit[1] if hit else 1)
            if not any(cross):
                continue
            c = m.new_bool_var(f"{name}_cross{ridx}")
            m.add_element(idx, cross, c)
            vin = m.new_int_var(0, H, "")
            vout = m.new_int_var(0, H, "")
            m.add_element(idx, tin, vin)
            m.add_element(idx, tout, vout)
            vsize = m.new_int_var(1, H, "")
            m.add(vsize == vout - vin)
            sv = m.new_int_var(0, 2 * H, "")
            m.add(sv == dep + vin)
            ev_ = m.new_int_var(0, 2 * H, "")
            m.add(ev_ == dep + vout)
            region_ivs[ridx].append(
                m.new_optional_interval_var(sv, vsize, ev_, both(lit, c, f"{name}_occlit{ridx}"), f"{name}_occ{ridx}")
            )

    if occupancy:
        for n, a in enumerate(nodes):
            a["arr"] = m.new_int_var(0, H, f"arr{n}")
            a["dep"] = m.new_int_var(0, H, f"dep{n}")
            pres = a["present"]
            if a["kind"] == "service":
                m.add(a["arr"] <= a["s"])
                m.add(a["dep"] >= a["e"])
            else:
                m.add(a["arr"] <= a["s"]).only_enforce_if(pres)
                m.add(a["dep"] >= a["e"]).only_enforce_if(pres)
            size = m.new_int_var(0, H, "")
            m.add(size == a["dep"] - a["arr"])
            if isinstance(a["loc"], int):
                occ.setdefault(a["loc"], []).append(
                    m.new_interval_var(a["arr"], size, a["dep"], f"occ{n}")
                    if a["kind"] == "service"
                    else m.new_optional_interval_var(a["arr"], size, a["dep"], pres, f"occ{n}")
                )
            else:
                for v in range(G):
                    at_v = m.new_bool_var(f"node{n}_at{v}")
                    m.add(a["loc"] == v).only_enforce_if(at_v)
                    m.add(a["loc"] != v).only_enforce_if(at_v.Not())
                    lit = m.new_bool_var("")
                    m.add_bool_and([pres, at_v]).only_enforce_if(lit)
                    m.add_bool_or([pres.Not(), at_v.Not()]).only_enforce_if(lit.Not())
                    occ.setdefault(v, []).append(
                        m.new_optional_interval_var(a["arr"], size, a["dep"], lit, f"occ{n}_{v}")
                    )
            if regions:
                rsize = m.new_int_var(0, H, "")
                m.add(rsize == a["dep"] - a["arr"])
                if isinstance(a["loc"], int):
                    for ridx in regions["inside"].get(a["loc"], []):
                        region_ivs[ridx].append(
                            m.new_interval_var(a["arr"], rsize, a["dep"], f"node{n}_reg{ridx}")
                            if a["kind"] == "service"
                            else m.new_optional_interval_var(a["arr"], rsize, a["dep"], pres, f"node{n}_reg{ridx}")
                        )
                else:
                    for ridx in range(regions["n"]):
                        table = [1 if ridx in regions["inside"].get(l, []) else 0 for l in range(G)]
                        if not any(table):
                            continue
                        c = m.new_bool_var(f"node{n}_inside{ridx}")
                        m.add_element(a["loc"], table, c)
                        region_ivs[ridx].append(
                            m.new_optional_interval_var(
                                a["arr"],
                                rsize,
                                a["dep"],
                                c if a["kind"] == "service" else both(pres, c, f"node{n}_insidelit{ridx}"),
                                f"node{n}_reg{ridx}",
                            )
                        )
    for r, robot in enumerate(robots):
        available = [n for n, a in enumerate(nodes) if tasks[a["task"]]["type"] in robot["durations"]]
        arcs, intervals = [], []
        idle = m.new_bool_var(f"robot{r}_idle")
        arcs.append((0, 0, idle))
        if occupancy:
            dep0 = m.new_int_var(0, H, f"dep0_{r}")
            occ.setdefault(G + r, []).append(m.new_interval_var(0, dep0, dep0, f"start_occ{r}"))
            m.add(dep0 == H).only_enforce_if(idle)
            arr_park = m.new_int_var(0, H, f"arr_park{r}")
            park[r] = {"dep0": dep0, "arr_park": arr_park, "choices": []}
        memberships = []
        for n in available:
            a = nodes[n]
            chosen = m.new_bool_var(f"robot{r}_node{n}")
            assigned = m.new_bool_var(f"robot{r}_owner{n}")
            m.add(owners[a["task"]] == r).only_enforce_if(assigned)
            m.add(owners[a["task"]] != r).only_enforce_if(assigned.Not())
            m.add(chosen <= assigned)
            m.add(chosen <= a["present"])
            m.add(chosen >= assigned + a["present"] - 1)
            memberships.append(chosen)
            arcs.append((n + 1, n + 1, chosen.Not()))
            first, last = m.new_bool_var(f"r{r}_first{n}"), m.new_bool_var(f"r{r}_last{n}")
            arcs.extend([(0, n + 1, first), (n + 1, 0, last)])
            loc = a["loc"]
            if isinstance(loc, int):
                initial = startdist[r][loc]
            else:
                initial = m.new_int_var(0, max(startdist[r]), f"r{r}_initdist{n}")
                m.add_element(loc, startdist[r], initial)
            m.add(a["s"] >= initial).only_enforce_if(first)
            if occupancy:
                m.add(a["arr"] >= dep0 + initial).only_enforce_if(first)
                add_crossings(dep0, first, ("start", r), loc, f"r{r}_start_{n}")
                # park choices after this node when it is the robot's last one
                chs = []
                cand = [(v, dist[loc][v] if isinstance(loc, int) else None) for v in range(G)] + [
                    (G + r, startdist[r][loc] if isinstance(loc, int) else None)
                ]
                for v, tr in cand:
                    if tr is None:
                        continue
                    c = m.new_bool_var(f"r{r}_park{n}_{v}")
                    chs.append(c)
                    park[r]["choices"].append((c, n, v, tr))
                    m.add(arr_park >= a["dep"] + tr).only_enforce_if(c)
                    if regions and v != loc:
                        add_crossings(a["dep"], c, loc, (v if v < G else ("start", r)), f"r{r}_parkmove{n}_{v}")
                    if regions:
                        for ridx in regions["inside"].get(v, []) if v < G else regions["inside_start"].get(r, []):
                            psz = m.new_int_var(0, H, "")
                            m.add(psz == H - arr_park)
                            region_ivs[ridx].append(
                                m.new_optional_interval_var(arr_park, psz, H, c, f"r{r}_park{n}_{v}_reg{ridx}")
                            )
                    if v == loc:
                        m.add(arr_park == a["dep"]).only_enforce_if(c)
                    sz = m.new_int_var(0, H, "")
                    m.add(sz == H - arr_park)
                    occ.setdefault(v, []).append(m.new_optional_interval_var(arr_park, sz, H, c, f"park{r}_{n}_{v}"))
                if chs:
                    m.add(sum(chs) == 1).only_enforce_if(last)
                    m.add(sum(chs) == 0).only_enforce_if(last.Not())
                else:
                    m.add(last == 0)
            duration = m.new_int_var(0, H, f"r{r}_duration{n}")
            m.add(duration == a["e"] - a["s"])
            intervals.append(m.new_optional_interval_var(a["s"], duration, a["e"], chosen, f"r{r}_interval{n}"))
        m.add(sum(memberships) == 0).only_enforce_if(idle)
        m.add(sum(memberships) >= 1).only_enforce_if(idle.Not())
        for n in available:
            for q in available:
                if n == q:
                    continue
                arc = m.new_bool_var(f"r{r}_arc{n}_{q}")
                arcs.append((n + 1, q + 1, arc))
                m.add(nodes[q]["s"] >= nodes[n]["e"] + travel(n, q)).only_enforce_if(arc)
                if occupancy:
                    m.add(nodes[q]["arr"] >= nodes[n]["dep"] + travel(n, q)).only_enforce_if(arc)
                    add_crossings(nodes[n]["dep"], arc, nodes[n]["loc"], nodes[q]["loc"], f"r{r}_{n}_{q}")
                robot_arc_count += 1
        m.add_circuit(arcs)
        m.add_no_overlap(intervals)
    for g in range(len(goals)):
        intervals = []
        for j, t in enumerate(tasks):
            if t["goal"] == g:
                a = nodes[service[j]]
                d = m.new_int_var(1, H, f"goal_duration{j}")
                m.add(d == a["e"] - a["s"])
                intervals.append(m.new_interval_var(a["s"], d, a["e"], f"goal_service{j}"))
        m.add_no_overlap(intervals)
    for i, j in instance.get("precedence", []):
        m.add(nodes[service[j]]["s"] >= nodes[service[i]]["e"])
    if occupancy:
        for v, ivs in occ.items():
            if len(ivs) > 1:
                m.add_no_overlap(ivs)
        for ridx, ivs in region_ivs.items():
            if len(ivs) > 1:
                m.add_no_overlap(ivs)
    end = m.new_int_var(0, H, "makespan")
    m.add_max_equality(end, [a["e"] for a in nodes] + ([p["arr_park"] for p in park.values()] if occupancy else []))
    if deadline is not None:
        m.add(end <= ticks(deadline))
    m.minimize(end)
    model_error = m.validate()
    if model_error:
        raise ValueError(model_error)
    solver = cp_model.CpSolver()
    gap_policy = gap_limit is not None
    if gap_policy:
        first_after_base = False
        limit = cp_max_s if cp_max_s is not None else seconds
        if wall_deadline:
            limit = min(limit, wall_deadline - time.monotonic())
        solver.parameters.max_time_in_seconds = max(0.001, limit)
        solver.parameters.relative_gap_limit = gap_limit
    else:
        solver.parameters.max_time_in_seconds = (
            max(0.001, wall_deadline - time.monotonic()) if wall_deadline else seconds
        )
        solver.parameters.relative_gap_limit = 0
    solver.parameters.absolute_gap_limit = 0
    solver.parameters.num_search_workers = workers
    solver.parameters.random_seed = seed
    build_seconds = time.perf_counter() - build_start
    updates = []
    solve_start = time.monotonic()
    done = threading.Event()
    have_solution = threading.Event()

    class Capture(cp_model.CpSolverSolutionCallback):
        def on_solution_callback(self):
            elapsed = time.monotonic() - solve_start
            updates.append(
                dict(
                    solver_elapsed_s=elapsed,
                    objective=self.objective_value / SCALE,
                    bound=self.best_objective_bound / SCALE,
                )
            )
            have_solution.set()
            if first_after_base and elapsed >= seconds:
                self.stop_search()

    def stop_at_base():
        if not done.wait(seconds) and have_solution.is_set():
            solver.stop_search()

    timer = None
    if first_after_base:
        timer = threading.Thread(target=stop_at_base, daemon=True)
        timer.start()
    try:
        status = solver.solve(m, Capture())
    finally:
        done.set()
        if timer:
            timer.join()

    result = dict(
        name=instance["name"],
        status=solver.status_name(status),
        solver_seconds=solver.wall_time,
        build_seconds=build_seconds,
        time_limit=(cp_max_s if gap_policy else seconds),
        gap_limit=gap_limit,
        workers=workers,
        seed=seed,
        model_variables=len(m.proto.variables),
        model_constraints=len(m.proto.constraints),
        robot_arc_count=robot_arc_count,
        scale=SCALE,
        incumbent_updates=updates,
        policy=(
            ("gap%.3f_max%ss" % (gap_limit, cp_max_s))
            if gap_policy
            else ("5s_then_first_feasible" if first_after_base else "fixed_time")
        ),
        extended=bool(first_after_base and (not updates or updates[0]["solver_elapsed_s"] >= seconds)),
        scope="abstract tool-aware schedule; no robot collision validation",
    )
    if status in [cp_model.FEASIBLE, cp_model.OPTIMAL]:
        result.update(
            makespan=solver.value(end) / SCALE,
            lower_bound=solver.best_objective_bound / SCALE,
            relative_gap=(solver.objective_value - solver.best_objective_bound) / max(solver.objective_value, 1e-9),
            gap_reached=bool(
                gap_policy
                and (solver.objective_value - solver.best_objective_bound) / max(solver.objective_value, 1e-9)
                <= gap_limit + 1e-9
            ),
        )
        actions = []
        for n, a in enumerate(nodes):
            if solver.value(a["present"]):
                actions.append(
                    dict(
                        node=n,
                        kind=a["kind"],
                        task=a["task"],
                        robot=solver.value(owners[a["task"]]),
                        goal=solver.value(a["loc"]),
                        start=solver.value(a["s"]),
                        end=solver.value(a["e"]),
                        **({"arr": solver.value(a["arr"]), "dep": solver.value(a["dep"])} if occupancy else {}),
                    )
                )
        if occupancy:
            result["parking"] = []
            for r, p in park.items():
                ch = [(n, v, tr) for c, n, v, tr in p["choices"] if solver.value(c)]
                if len(ch) == 1:
                    n, v, tr = ch[0]
                    t_arr = solver.value(p["arr_park"])
                    t_dep = solver.value(nodes[n]["dep"])
                    result["parking"].append(
                        dict(robot=r, after_node=n, vertex=v, stay=(v == nodes[n]["loc"]), depart=t_dep, arrive=t_arr)
                    )
                    if v != nodes[n]["loc"]:
                        actions.append(dict(node=-1, kind="park", task=None, robot=r, goal=v, start=t_dep, end=t_arr))
            result["occupancy"] = {"vertices": len(occ), "intervals": sum(len(x) for x in occ.values())}
            result["start_departure"] = {r: solver.value(p["dep0"]) for r, p in park.items()}
            result["regions"] = regions["n"] if regions else 0
            result["region_intervals"] = sum(len(v) for v in region_ivs.values())
        result["actions"] = sorted(actions, key=lambda a: (a["start"], a["robot"], a["end"]))
        result["tool_arcs"] = [list(k) for k, v in tool_selected.items() if solver.value(v)]
        result["validation_errors"] = validate(instance, result)
        result["validated"] = not result["validation_errors"]
    return result


def validate(instance, result):
    """Independent chronological replay of the schedule."""
    errors, actions = [], result["actions"]
    robots, goals, tasks = (instance[k] for k in ["robots", "goals", "tasks"])
    locations = instance.get("locations", goals)
    services = {}
    for a in actions:
        if a["start"] < 0 or a["end"] <= a["start"]:
            errors.append("Nonpositive duration or negative start")
        if a["kind"] == "service":
            j = a["task"]
            if j in services:
                errors.append(f"Duplicate service {j}")
            services[j] = a
            typ = tasks[j]["type"]
            expected = robots[a["robot"]]["durations"].get(typ)
            if expected is None or a["end"] - a["start"] != ticks(expected):
                errors.append(f"Capability/service duration {j}")
            if a["goal"] not in tasks[j].get("locations", [tasks[j]["goal"]]):
                errors.append(f"Service location {j}")
        elif a["kind"] in ["pickup", "drop"]:
            if a["end"] - a["start"] != ticks(instance["handling_seconds"]):
                errors.append("Handling duration")
        elif a["kind"] == "park":
            pass
        else:
            errors.append("Unknown action")
    if set(services) != set(range(len(tasks))):
        errors.append("Missing/extra service")
    for r, robot in enumerate(robots):
        pos, end, previous_location = robot["start"], 0, None
        for a in sorted((a for a in actions if a["robot"] == r), key=lambda a: a["start"]):
            if a["kind"] == "park":
                G_ = len(locations)
                dest = robots[a["goal"] - G_]["start"] if a["goal"] >= G_ else locations[a["goal"]]["position"]
                if "travel_ticks" in instance and previous_location is not None:
                    need = (
                        instance["start_travel_ticks"][a["goal"] - G_][previous_location]
                        if a["goal"] >= G_
                        else instance["travel_ticks"][previous_location][a["goal"]]
                    )
                else:
                    need = ticks(math.dist(pos, dest) / instance["speed"])
                if a["end"] < end + need - 1:
                    errors.append(f"Robot {r} park travel shortage")
                pos, end = dest, a["end"]
                continue
            dest = locations[a["goal"]]["position"]
            need = ticks(math.dist(pos, dest) / instance["speed"])
            if "travel_ticks" in instance:
                need = (
                    instance["start_travel_ticks"][r][a["goal"]]
                    if previous_location is None
                    else instance["travel_ticks"][previous_location][a["goal"]]
                )
            if a["start"] < end + need:
                errors.append(f"Robot {r} overlap/travel shortage")
            pos, end, previous_location = dest, a["end"], a["goal"]
    for g in range(len(goals)):
        end = 0
        for a in sorted(
            (a for a in actions if a["kind"] == "service" and tasks[a["task"]]["goal"] == g), key=lambda a: a["start"]
        ):
            if a["start"] < end:
                errors.append(f"Goal {g} service overlap")
            end = a["end"]
    for i, j in instance.get("precedence", []):
        if i in services and j in services and services[j]["start"] < services[i]["end"]:
            errors.append(f"Precedence {i}->{j}")
    # the tool is unavailable while handled and owned during tool service
    owner, free_goal, busy = None, instance.get("tool_initial_location", instance["tool_initial_goal"]), None
    active_tool_services = set()
    events = [(a["start"], 1, idx, a) for idx, a in enumerate(actions) if a["kind"] != "park"]
    events += [(a["end"], 0, idx, a) for idx, a in enumerate(actions) if a["kind"] != "park"]
    for _, is_start, idx, a in sorted(events, key=lambda x: x[:3]):
        r, kind = a["robot"], a["kind"]
        tool_service = kind == "service" and tasks[a["task"]]["tool"]
        if is_start:
            if kind == "pickup":
                if owner is not None or busy is not None or free_goal != a["goal"]:
                    errors.append(f'Illegal pickup by robot {r} at goal {a["goal"]}')
                busy = ("pickup", idx)
                free_goal = None
            elif kind == "drop":
                if owner != r or busy is not None or active_tool_services:
                    errors.append(f"Illegal drop by robot {r}")
                busy = ("drop", idx)
            elif tool_service:
                if owner != r or busy is not None or active_tool_services:
                    errors.append(f'Tool condition violated in task {a["task"]}')
                active_tool_services.add(idx)
        else:
            if kind == "pickup":
                if busy != ("pickup", idx):
                    errors.append("Pickup completion mismatch")
                owner, busy = r, None
            elif kind == "drop":
                if busy != ("drop", idx):
                    errors.append("Drop completion mismatch")
                owner, free_goal, busy = None, a["goal"], None
            elif tool_service:
                if owner != r or busy is not None:
                    errors.append("Tool not held throughout service")
                active_tool_services.discard(idx)
    if abs(max((a["end"] for a in actions), default=0) / SCALE - result["makespan"]) > 1e-8:
        errors.append("Makespan mismatch")
    return errors


def tiny_instances():
    single = dict(
        name="tiny_keep_tool",
        robots=[dict(name="r0", start=[0, 0], durations={"a": 2})],
        goals=[dict(name="g0", position=[0, 0]), dict(name="g1", position=[3, 0])],
        tasks=[dict(name="j0", goal=0, type="a", tool=True), dict(name="j1", goal=1, type="a", tool=True)],
        tool_initial_goal=0,
        handling_seconds=0.5,
        speed=1.0,
        precedence=[[0, 1]],
    )
    handoff = copy.deepcopy(single)
    handoff.update(
        name="tiny_forced_handoff",
        robots=[dict(name="r0", start=[0, 0], durations={"a": 2}), dict(name="r1", start=[3, 0], durations={"b": 1})],
    )
    handoff["tasks"][1]["type"] = "b"
    samegoal = copy.deepcopy(handoff)
    samegoal.update(name="tiny_same_goal")
    samegoal["goals"][1]["position"] = [0, 0]
    samegoal["robots"][1]["start"] = [0, 0]
    # both services use the same goal to exercise NoOverlap
    samegoal["tasks"][1]["goal"] = 0
    assignment = copy.deepcopy(single)
    assignment.update(
        name="tiny_assignment_choice",
        precedence=[],
        robots=[
            dict(name="fast", start=[0, 0], durations={"a": 1}),
            dict(name="slow", start=[0, 0], durations={"a": 10}),
        ],
    )
    carry = copy.deepcopy(single)
    carry["name"] = "tiny_non_tool_service_while_carrying"
    carry["goals"].append(dict(name="middle", position=[1.5, 0]))
    carry["robots"][0]["durations"]["c"] = 1
    carry["tasks"].insert(1, dict(name="ordinary", goal=2, type="c", tool=False))
    carry["precedence"] = [[0, 1], [1, 2]]
    return [(single, 7.5), (handoff, 7.5), (samegoal, 4.5), (assignment, 5.5), (carry, 8.5)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("inputs", nargs="*", type=Path)
    p.add_argument("--seconds", type=float, default=5)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--output", type=Path, default=Path("results"))
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    if args.self_test:
        for case, expected in tiny_instances():
            res = solve(case, seconds=5, workers=1)
            assert res["status"] == "OPTIMAL", res
            assert res["validated"], res
            assert abs(res["makespan"] - expected) < 1e-8, (expected, res)
            if case["name"] == "tiny_keep_tool":
                assert sum(a["kind"] == "pickup" for a in res["actions"]) == 1
                assert not any(a["kind"] == "drop" for a in res["actions"])
            mutated = copy.deepcopy(res)
            mutated["actions"] = [a for a in mutated["actions"] if a["kind"] != "pickup"]
            assert validate(case, mutated), "Validator accepted removed pickup"
            mutated = copy.deepcopy(res)
            next(a for a in mutated["actions"] if a["kind"] == "pickup")["goal"] = 1
            if case["name"] != "tiny_same_goal":
                assert validate(case, mutated), "Validator accepted wrong pickup location"
            impossible = solve(case, seconds=5, workers=1, deadline=expected - 0.01)
            assert impossible["status"] == "INFEASIBLE", impossible
            res["checks"] = dict(
                expected_optimum=expected,
                impossible_deadline="INFEASIBLE",
                removed_pickup_rejected=True,
                expected_optimum_verified=True,
            )
            (args.output / f'{case["name"]}.json').write_text(json.dumps(dict(instance=case, result=res), indent=2))
            results.append(res)
            print(json.dumps({k: v for k, v in res.items() if k not in ["actions", "tool_arcs"]}), flush=True)
    for path in args.inputs:
        instance = load_yaml(path) if path.suffix in [".yaml", ".yml"] else json.loads(path.read_text())
        result = solve(instance, seconds=args.seconds, workers=args.workers, seed=args.seed)
        (args.output / f'{instance["name"]}.json').write_text(
            json.dumps(dict(instance=instance, result=result), indent=2)
        )
        if result.get("validation_errors"):
            raise AssertionError(result["validation_errors"])
        results.append(result)
        print(json.dumps({k: v for k, v in result.items() if k not in ["actions", "tool_arcs"]}), flush=True)
    (args.output / "summary.json").write_text(
        json.dumps([{k: v for k, v in r.items() if k not in ["actions", "tool_arcs"]} for r in results], indent=2)
    )


if __name__ == "__main__":
    main()
