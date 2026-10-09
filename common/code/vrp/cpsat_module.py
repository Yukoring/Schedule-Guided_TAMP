"""CP-SAT relaxation oracle: flexible job shop with travel times.

Drop-in alternative to ORToolsModule (same solver signature and vrp_info
guidance keys). Tasks are optional intervals with (robot, goal-waypoint)
alternatives; each robot's route is an AddCircuit over its selected
alternatives with travel-time arcs. Compared to the routing encoding this
gives exact semantics the routing model only approximated:

- precedence is finish-before-start on per-task master variables (the
 routing model bound one split copy's completion +1.2s);
- one-robot-per-goal-at-a-time is a NoOverlap per goal (the routing model
 compared per-vehicle visit counters and patched overlaps afterwards);
- makespan is the objective of a single solve (no tightening loop);
- deadline margins derive from robot_diameter/velocity instead of the
 hand-tuned 50/120 constants.

Collision waypoints carry no tasks and are ignored as schedule nodes; like
the routing relaxation, geometry stays the motion planner's job. The
region-aware subclass (CpSatRegionModule, --oracle cpsat2) additionally
models narrow-region occupancy: every travel arc whose roadmap path
crosses a narrow collision region gets an explicit departure variable and
an optional occupancy interval over the crossing window, and each region
enforces NoOverlap across all robots - a time-aware mutual exclusion the
routing VRP cannot express (it allowed simultaneous occupancy and patched
timelines afterwards).
"""

import math

from ortools.sat.python import cp_model

VRP_TIME_LIMIT = 5  # seconds, same budget as the routing oracle
SCALE = 100  # integer time units per second (matches or_module)


class CpSatModule:
    region_aware = False

    def __init__(self, vrp_instance, margin_scale=1.0, region_exclusion=True):
        # ICAR contention experiment control: when False, ONLY the per-
        # region AddNoOverlap constraints are skipped. Regions, tokens,
        # transient input, through/detour arcs, departure variables,
        # occupancy interval construction and their time links, the
        # workstation NoOverlap, precedences, and the objective all stay.
        self.region_exclusion = region_exclusion
        # Adaptive deadline slack: the caller widens this when the
        # deadline-guided variant failed to plan in the previous
        # iteration (relaxation optimism vs realizable times).
        self.margin_scale = margin_scale
        self.debug = vrp_instance.env.debug_mode
        self.obj_ins = vrp_instance.obj_ins
        self.velocity = getattr(vrp_instance, "velocity", 1.0)
        self.waypoints = vrp_instance.obj_ins["waypoint"]
        self.robots = vrp_instance.obj_ins["robot"]
        self.jobs = vrp_instance.obj_ins["job"]
        self.tasks = vrp_instance.obj_ins["task"]

        self.distance = {}  # root -> {root: geometric path length}
        self.paths = {}  # root -> {root: [pts]}
        for ele in self.waypoints:
            self.distance[ele.ind] = {}
            self.paths[ele.ind] = {}
            for dest, pts in zip(ele.connected, ele.path):
                if not pts or pts[0] is None:
                    # A cost-only edge cannot supply region occupancy or
                    # an executable dispatch; do not offer it to the model.
                    continue
                self.paths[ele.ind][dest] = list(pts)
                self.distance[ele.ind][dest] = self._path_length(pts)
        # Close the matrix over the waypoint graph (Floyd-Warshall; the
        # graph is tiny). Direct edges only made the relaxation go blind
        # on any goal swallowed by a collision region: normal<->col pairs
        # have no direct edge by design, every (robot, goal) alternative
        # got filtered, and the guidance collapsed to empty for the rest
        # of the run ('no robot can perform ...'). Compose the geometric
        # path with its cost: every offered travel arc must expose ALL
        # crossed regions and must later execute that same path.
        inds = [w.ind for w in self.waypoints]
        for k in inds:
            dk = self.distance.get(k, {})
            for a in inds:
                da = self.distance.get(a, {})
                ak = da.get(k)
                if ak is None:
                    continue
                for b, kb in dk.items():
                    alt = ak + kb
                    if b != a and (b not in da or alt < da[b]):
                        left = self.paths[a][k]
                        right = self.paths[k][b]
                        if tuple(left[-1]) != tuple(right[0]):
                            raise ValueError("Disconnected geometry in waypoint closure")
                        da[b] = alt
                        self.paths[a][b] = left + right[1:]
                self.distance[a] = da

        # Collision regions (used when region_aware): both narrow and
        # normal regions become time-exclusive resources - through-traffic
        # occupies them for its geometric crossing window.
        self.regions = []  # list of (waypoint-ind set, sample list)
        self.region_radius = max((r.radius for r in self.robots), default=0.3) * 2
        # NOTE: with a size-uniform fleet the per-region NoOverlap is the
        # unit-capacity case of a cumulative resource; a width-capacitated
        # AddCumulative generalization (demands = robot diameters,
        # capacity = 2*throat) was implemented and gated behavior-identical
        # for uniform radii - dropped as code, kept as the one-line
        # generalization remark in the paper (git 79ca074).
        for token in vrp_instance.obj_ins.get("token", []):
            if token.token_name in ("narrow_col_token", "normal_col_token") and token.token_samples:
                self.regions.append((set(token.token_wp), token.token_samples))
        # Transient crossing conflicts (open-space, first occurrence): they
        # never reach the PDDL encoding - the schedule alone separates the
        # crossing times, so no waypoint set is attached.
        for samples in getattr(vrp_instance, "transient_regions", []):
            if samples:
                self.regions.append((set(), samples))
        self.prm = getattr(vrp_instance, "prm", None)
        self._loc = {w.ind: w.loc for w in self.waypoints}
        self._through_cache = {}
        self._crossing_cache = {}

    @staticmethod
    def _path_length(pts):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))

    def _path_time_units(self, pts):
        # Reserve whole centiseconds, then preserve physical travel time
        # in dispatch. This prevents rounding drift in generated paths.
        return int(math.ceil(self._path_length(pts) / self.velocity * SCALE - 1e-9))

    def _crossings(self, w1, w2):
        """Regions the stored roadmap path w1->w2 crosses (time offsets in
        integer units relative to departure)."""
        key = (w1, w2)
        if key in self._crossing_cache:
            return self._crossing_cache[key]
        pts = self.paths.get(w1, {}).get(w2)
        if not pts or pts[0] is None:
            if w1 != w2 and w2 in self.distance.get(w1, {}):
                raise ValueError("Reachable travel arc is missing geometry")
            return []
        self._crossing_cache[key] = self._crossings_pts(pts)
        return self._crossing_cache[key]

    def _through(self, w1, w2):
        """Optional through-region travel: after env_update the stored
        distances detour around normal collision regions, so the schedule
        would never see the crossing option. Query the original task roadmap
        for the through path; if it crosses a region, offer it as an
        alternative arc that occupies the region. Cached per pair."""
        key = (w1, w2)
        if key in self._through_cache:
            return self._through_cache[key]
        res = None
        if self.prm is not None and w1 != w2:
            path, cost = self.prm.QueryTaskPlan(self._loc[w1], self._loc[w2])
            if path is not None and cost != -1:
                cross = self._crossings_pts(path)
                if cross:
                    res = (self._path_time_units(path), cross, path)
        self._through_cache[key] = res
        return res

    def _crossings_pts(self, pts):
        # segment-based region entry and exit
        out = []
        segs = []
        cum = 0.0
        for a, b in zip(pts, pts[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            segs.append((a, b, cum, seg_len))
            cum += seg_len
        if not segs:
            segs = [(pts[0], pts[0], 0.0, 0.0)]
        r = self.region_radius
        for ridx, (wps, samples) in enumerate(self.regions):
            d_in = d_out = None
            for a, b, base, seg_len in segs:
                if seg_len <= 1e-9:
                    if any(math.hypot(a[0] - sx, a[1] - sy) < r for sx, sy in samples):
                        d_in = base if d_in is None else min(d_in, base)
                        d_out = base if d_out is None else max(d_out, base)
                    continue
                dx = (b[0] - a[0]) / seg_len
                dy = (b[1] - a[1]) / seg_len
                for sx, sy in samples:
                    fx, fy = a[0] - sx, a[1] - sy
                    bq = fx * dx + fy * dy
                    disc = bq * bq - (fx * fx + fy * fy - r * r)
                    if disc < 0:
                        continue
                    sq = math.sqrt(disc)
                    t0, t1 = -bq - sq, -bq + sq
                    if t1 < 0 or t0 > seg_len:
                        continue
                    e_in = base + max(t0, 0.0)
                    e_out = base + min(t1, seg_len)
                    d_in = e_in if d_in is None else min(d_in, e_in)
                    d_out = e_out if d_out is None else max(d_out, e_out)
            if d_in is not None:
                t_in = int(math.floor(d_in / self.velocity * SCALE + 1e-9))
                t_out = int(math.ceil(d_out / self.velocity * SCALE - 1e-9))
                if t_out <= t_in:
                    t_out = t_in + 1
                out.append((ridx, t_in, t_out))
        return out

    def _travel(self, w1, w2):
        """Integer travel time between waypoint roots; None if unreachable."""
        if w1 == w2:
            return 0
        d = self.distance.get(w1, {}).get(w2)
        if d is None:
            return None
        return int(math.ceil(d / self.velocity * SCALE - 1e-9))

    def solver(self):
        model = cp_model.CpModel()
        horizon = 0

        # (job, task) master steps and their (robot, waypoint) alternatives
        steps = {}  # (job_ind, task_ind) -> dict
        for task in self.tasks:
            for job_ind in task.loc:
                steps[(job_ind, task.ind)] = {"alts": []}
        job_by_ind = {j.ind: j for j in self.jobs}

        for robot in self.robots:
            for (job_ind, task_ind), step in steps.items():
                if task_ind not in robot.action_duration:
                    continue
                dur = int(round(robot.action_duration[task_ind] * SCALE))
                for w in job_by_ind[job_ind].located:
                    root = self._root_of(w)
                    if self._travel(robot.at[0], root) is None:
                        continue
                    horizon += dur + SCALE * 100
                    step["alts"].append({"robot": robot, "w": root, "dur": dur})
        horizon = max(horizon, 1)

        for (job_ind, task_ind), step in steps.items():
            if not step["alts"]:
                if self.debug:
                    print("CpSat: no robot can perform", task_ind, "at", job_ind)
                return None, -1, self._empty_info()
            step["start"] = model.NewIntVar(0, horizon, "s_%s_%s" % (job_ind, task_ind))
            step["end"] = model.NewIntVar(0, horizon, "e_%s_%s" % (job_ind, task_ind))
            for k, alt in enumerate(step["alts"]):
                alt["lit"] = model.NewBoolVar("p_%s_%s_%d" % (job_ind, task_ind, k))
                alt["start"] = model.NewIntVar(0, horizon, "")
                alt["end"] = model.NewIntVar(0, horizon, "")
                alt["interval"] = model.NewOptionalIntervalVar(alt["start"], alt["dur"], alt["end"], alt["lit"], "")
                model.Add(step["start"] == alt["start"]).OnlyEnforceIf(alt["lit"])
                model.Add(step["end"] == alt["end"]).OnlyEnforceIf(alt["lit"])
            model.AddExactlyOne(alt["lit"] for alt in step["alts"])

        # one robot works at a goal at a time (work_free semantics)
        for job in self.jobs:
            ivs = [alt["interval"] for (j, t), step in steps.items() if j == job.ind for alt in step["alts"]]
            if len(ivs) > 1:
                model.AddNoOverlap(ivs)

        # precedence: order[0] finishes before order[1] starts
        for job in self.jobs:
            for order in job.order:
                a = steps.get((job.ind, order[0]))
                b = steps.get((job.ind, order[1]))
                if a and b:
                    model.Add(b["start"] >= a["end"])

        # per-robot route: circuit over its alternatives with travel arcs
        region_ivs = {i: [] for i in range(len(self.regions))}
        if self.region_aware:
            # a task performed inside a narrow region occupies it for its duration
            for step in steps.values():
                for alt in step["alts"]:
                    for ridx, (wps, samples) in enumerate(self.regions):
                        if alt["w"] in wps:
                            region_ivs[ridx].append(alt["interval"])

        def add_occupancy(model, lit, dep_lb_expr, arrive_start, travel, cross):
            """Departure var + occupancy interval for a crossing arc."""
            dep = model.NewIntVar(0, horizon, "")
            model.Add(dep >= dep_lb_expr).OnlyEnforceIf(lit)
            model.Add(arrive_start >= dep + travel).OnlyEnforceIf(lit)
            for ridx, t_in, t_out in cross:
                iv = model.NewOptionalIntervalVar(dep + t_in, t_out - t_in, dep + t_out, lit, "")
                region_ivs[ridx].append(iv)
            return dep

        # remember every guided arc so that extraction reproduces the solver departure times and geometry
        self._arc_records = []

        def record_arc(robot, to_key, from_w, lit, dep, travel, pts, cross):
            self._arc_records.append(
                {
                    "robot": robot.ind,
                    "to": to_key,
                    "from_w": from_w,
                    "lit": lit,
                    "dep": dep,
                    "travel": travel,
                    "pts": pts,
                    "cross": list(cross),
                }
            )

        for robot in self.robots:
            nodes, node_keys = [], []
            for jt, step in steps.items():
                for alt in step["alts"]:
                    if alt["robot"] is robot:
                        nodes.append(alt)
                        node_keys.append(jt)
            if not nodes:
                continue
            arcs = []
            n = len(nodes)
            for i, alt in enumerate(nodes):
                arcs.append((0, i + 1, model.NewBoolVar("")))  # depart start
                arcs.append((i + 1, 0, model.NewBoolVar("")))  # end route
                skip = alt["lit"].Not()
                arcs.append((i + 1, i + 1, skip))  # unassigned
                t0 = self._travel(robot.at[0], alt["w"])
                model.Add(alt["start"] >= t0).OnlyEnforceIf(arcs[-3][2])
                cross = self._crossings(robot.at[0], alt["w"]) if self.region_aware else []
                dep = None
                if cross:
                    dep = add_occupancy(model, arcs[-3][2], 0, alt["start"], t0, cross)
                record_arc(
                    robot,
                    node_keys[i],
                    robot.at[0],
                    arcs[-3][2],
                    dep,
                    t0,
                    self.paths.get(robot.at[0], {}).get(alt["w"]),
                    cross,
                )
                if self.region_aware:
                    th = self._through(robot.at[0], alt["w"])
                    if th and (t0 is None or th[0] < t0):
                        lit2 = model.NewBoolVar("")
                        model.Add(alt["start"] >= th[0]).OnlyEnforceIf(lit2)
                        dep2 = add_occupancy(model, lit2, 0, alt["start"], th[0], th[1])
                        record_arc(robot, node_keys[i], robot.at[0], lit2, dep2, th[0], th[2], th[1])
                        arcs.append((0, i + 1, lit2))
            for i, a in enumerate(nodes):
                for j, b in enumerate(nodes):
                    if i == j:
                        continue
                    t = self._travel(a["w"], b["w"])
                    options = []
                    if t is not None:
                        cross = self._crossings(a["w"], b["w"]) if self.region_aware else []
                        options.append((t, cross, self.paths.get(a["w"], {}).get(b["w"])))
                    if self.region_aware and self.regions:
                        th = self._through(a["w"], b["w"])
                        if th and (t is None or th[0] < t):
                            options.append(th)
                    if not options:
                        lit = model.NewBoolVar("")
                        model.Add(lit == 0)
                        arcs.append((i + 1, j + 1, lit))
                        continue
                    for travel, cross, th_pts in options:
                        lit = model.NewBoolVar("")
                        model.Add(b["start"] >= a["end"] + travel).OnlyEnforceIf(lit)
                        if cross:
                            dep = add_occupancy(model, lit, a["end"], b["start"], travel, cross)
                        else:
                            dep = None
                        record_arc(robot, node_keys[j], a["w"], lit, dep, travel, th_pts, cross)
                        arcs.append((i + 1, j + 1, lit))
            arcs.append((0, 0, model.NewBoolVar("")))  # robot idle
            model.AddCircuit(arcs)

        self.last_stats = {"region_count": len(self.regions), "active_region_count": 0, "region_exclusion_count": 0}
        if self.region_aware:
            for ridx, ivs in region_ivs.items():
                if len(ivs) > 1:
                    self.last_stats["active_region_count"] += 1
                    if self.region_exclusion:
                        model.AddNoOverlap(ivs)
                        self.last_stats["region_exclusion_count"] += 1

        makespan = model.NewIntVar(0, horizon, "makespan")
        for step in steps.values():
            model.Add(makespan >= step["end"])
        model.Minimize(makespan)

        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = VRP_TIME_LIMIT
        solver.parameters.num_search_workers = 4
        solver.parameters.random_seed = 0
        import time as _t

        _s0 = _t.time()
        status = solver.Solve(model)
        self.last_stats.update({"cp_status": solver.StatusName(status), "cp_wall_s": round(_t.time() - _s0, 3)})
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            if self.debug:
                print("CpSat: no solution (status %s)" % solver.StatusName(status))
            return None, -1, self._empty_info()
        self.last_stats["cp_objective"] = solver.Value(makespan) / SCALE
        if self.debug:
            print("CpSat %s makespan=%.2f" % (solver.StatusName(status), solver.Value(makespan) / SCALE))
        return self._extract(solver, steps, solver.Value(makespan))

    def _root_of(self, wp_ind):
        return wp_ind  # located entries are waypoint inds already

    def _empty_info(self):
        return {
            "helpful_route": [],
            "goal_order": {},
            "goal_bound": {},
            "robot_task": {},
            "robot_help": {"start": [], "end": [], "time": []},
            "tours": {},
            "task_windows": {},
        }

    def _extract(self, solver, steps, makespan):
        info = self._empty_info()
        # Deadline slack grows with the task's position in its robot's tour:
        # the schedule's travel times are ideal, so later tasks accumulate
        # execution slippage (token waits, entry actions). Base unit is the
        # time to clear a workstation footprint, from geometry.
        max_diam = max((r.radius * 2 for r in self.robots), default=0.6)
        margin = max(int(round(max_diam / self.velocity * SCALE)), SCALE // 2)
        margin = int(round(margin * self.margin_scale))

        chosen = []  # (start, end, robot, w, job, task)
        for (job_ind, task_ind), step in steps.items():
            for alt in step["alts"]:
                if solver.Value(alt["lit"]):
                    chosen.append(
                        (
                            solver.Value(alt["start"]),
                            solver.Value(alt["end"]),
                            alt["robot"],
                            alt["w"],
                            job_ind,
                            task_ind,
                            alt["dur"],
                        )
                    )
        chosen.sort(key=lambda c: (c[0], c[1], c[2].ind))

        for r in self.robots:
            info["robot_task"][r.ind] = []
        per_robot = {}
        for start, end, robot, w, job_ind, task_ind, dur in chosen:
            if task_ind not in info["robot_task"][robot.ind]:
                info["robot_task"][robot.ind].append(task_ind)
            info["goal_order"][job_ind] = info["goal_order"].get(job_ind, []) + [task_ind]
            per_robot.setdefault(robot.ind, []).append((start, end, w, job_ind, task_ind, dur))

        for start, end, robot, w, job_ind, task_ind, dur in chosen:
            legs = per_robot[robot.ind]
            k = next(i for i, leg in enumerate(legs) if leg[0] == start)
            flag = 2 if k == 0 else 0
            info["goal_bound"][job_ind] = info["goal_bound"].get(job_ind, []) + [
                (task_ind, end + margin * (k + 1), flag)
            ]
            # capability window in seconds, slack grows with tour position
            slack = margin * (k + 1) / SCALE
            key = (robot.ind, task_ind)
            w1 = max(0.0, start / SCALE - slack)
            w2 = end / SCALE + slack
            if key in info["task_windows"]:
                w1 = min(w1, info["task_windows"][key][0])
                w2 = max(w2, info["task_windows"][key][1])
            info["task_windows"][key] = (w1, w2)

        self.last_stats["deadline_guidance"] = {
            "beta_s": margin / SCALE,
            "margin_scale": self.margin_scale,
            "T_hat_s": max(c[1] for c in chosen) / SCALE if chosen else None,
            "selected": [
                {"start_cs": start, "end_cs": end, "robot": robot.ind, "waypoint": w, "goal": job, "task": task}
                for start, end, robot, w, job, task, dur in chosen
            ],
            "source": "current schedule; no temporal reference plan",
        }

        path = {}
        tours = info["tours"]
        help_info = info["robot_help"]
        sel_arcs = {}
        for rec in getattr(self, "_arc_records", []):
            try:
                if solver.BooleanValue(rec["lit"]):
                    sel_arcs[(rec["robot"], rec["to"], rec["from_w"])] = rec
            except Exception:
                pass
        selected_geometry = []
        for robot in self.robots:
            path[robot.ind] = []
            legs = per_robot.get(robot.ind, [])
            curr_w = robot.at[0]
            curr_t = 0.0
            for i, (start, end, w, job_ind, task_ind, dur) in enumerate(legs):
                # legs are timed from the solver task start/end so that scheduled waits survive
                s_sec = start / SCALE
                if w != curr_w:
                    # use the solver's own arc choice (travel time, geometry, departure variable)
                    rec = sel_arcs.get((robot.ind, (job_ind, task_ind), curr_w))
                    if rec is None or not rec["pts"]:
                        raise ValueError("Selected travel arc has no recorded geometry")
                    t_int = rec["travel"]
                    pts = rec["pts"]
                    travel = self._path_length(pts) / self.velocity
                    if rec and rec["dep"] is not None:
                        depart = max(curr_t, solver.Value(rec["dep"]) / SCALE)
                    else:
                        depart = max(curr_t, s_sec - t_int / SCALE)
                    path[robot.ind].append([pts, depart, travel])
                    selected_geometry.append(
                        {
                            "robot": robot.ind,
                            "from": curr_w,
                            "to": w,
                            "depart_s": depart,
                            "travel_cs": t_int,
                            "physical_travel_s": travel,
                            "region_windows_cs": rec["cross"],
                            "region_intervals_s": [
                                [rid, depart + a / SCALE, depart + b / SCALE] for rid, a, b in rec["cross"]
                            ],
                            "path": pts,
                        }
                    )
                    if i == 0:
                        help_info["start"].append(curr_w)
                        help_info["end"].append(w)
                        help_info["time"].append(max(0, start - t_int))
                    if w not in tours.get(curr_w, []):
                        tours[curr_w] = tours.get(curr_w, []) + [w]
                    curr_t = depart + travel
                loc = next(ele.loc for ele in self.waypoints if ele.ind == w)
                svc_start = max(curr_t, s_sec)
                path[robot.ind].append([[loc], svc_start, dur / SCALE, task_ind, job_ind])
                curr_t = max(svc_start + dur / SCALE, end / SCALE)
                curr_w = w
            if legs:
                tours[curr_w] = tours.get(curr_w, []) + ["depot"]

        self.last_stats["selected_travel_geometry"] = selected_geometry
        return path, makespan / SCALE, info


class CpSatRegionModule(CpSatModule):
    """CP-SAT oracle with time-aware narrow-region mutual exclusion."""

    region_aware = True
