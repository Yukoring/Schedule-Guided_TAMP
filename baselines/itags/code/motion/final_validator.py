"""Final validation of a returned plan. Task side: service events are rebuilt from the executed timeline and checked against the
instance (capability and duration, precedence, one service at a time per goal, completeness). Motion side: continuous-time minimum
distance between every robot pair on the piecewise-linear trajectories, robots holding their final position until the end of the plan."""
import math

_EPS_T = 0.02      # temporal tolerance (matches the wait-preservation floor)
_EPS_D = 1e-9      # geometric slack for strict separation


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _timed_events(path_cost):
    """(events, services): events = [(pos, t)] breakpoints of a piecewise
    linear trajectory (repeated pos = hold); services = [(t0, t1, task, job)].
    """
    events, services = [], []
    t = 0.0

    def consume(pos, partials):
        nonlocal t
        for partial in partials:
            dur = partial[0]
            events.append((pos, t))
            if partial[1] != 2:               # (dur, task, job) service
                services.append((t, t + dur, partial[1], partial[2], pos))
            t += dur
        events.append((pos, t))

    if not path_cost:
        return events, services
    if len(path_cost) == 1:
        ele = path_cost[0]
        if isinstance(ele, list):
            consume(ele[0], ele[2])
        else:
            events.append((ele, 0.0))
        return events, services

    starts = None
    for i, ele in enumerate(path_cost):
        if i == 0:
            if isinstance(ele, list):
                consume(ele[0], ele[2])
                starts = ele[0]
            else:
                events.append((ele, t))
                starts = ele
            continue
        if isinstance(ele, list):
            pos = ele[0]
            t += _dist(starts, pos)
            events.append((pos, t))
            consume(pos, ele[2])
            starts = pos
        else:
            t += _dist(starts, ele)
            events.append((ele, t))
            starts = ele
    return events, services


def _pos_at(events, t):
    if t <= events[0][1]:
        return events[0][0]
    for (p0, t0), (p1, t1) in zip(events, events[1:]):
        if t <= t1:
            if t1 - t0 <= 1e-12:
                return p1
            a = (t - t0) / (t1 - t0)
            return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]))
    return events[-1][0]


def _pair_min_distance(ev1, ev2, horizon):
    """Continuous minimum distance over [0, horizon] for two piecewise
    linear trajectories that hold their last position."""
    times = sorted({t for _, t in ev1} | {t for _, t in ev2} | {0.0, horizon})
    times = [t for t in times if 0.0 <= t <= horizon + 1e-9]
    best = float('inf')
    where = None
    for ta, tb in zip(times, times[1:]):
        if tb - ta <= 1e-12:
            continue
        p0, p1 = _pos_at(ev1, ta), _pos_at(ev1, tb)
        q0, q1 = _pos_at(ev2, ta), _pos_at(ev2, tb)
        dt = tb - ta
        dx, dy = p0[0] - q0[0], p0[1] - q0[1]
        rx = (p1[0] - p0[0] - (q1[0] - q0[0])) / dt
        ry = (p1[1] - p0[1] - (q1[1] - q0[1])) / dt
        rr = rx * rx + ry * ry
        ts = 0.0 if rr <= 1e-15 else max(0.0, min(dt, -(dx * rx + dy * ry) / rr))
        for tq in (0.0, ts, dt):
            d = math.hypot(dx + rx * tq, dy + ry * tq)
            if d < best:
                best = d
                where = ta + tq
    return best, where


def validate_final(robot_path, obj_ins, robots_map):
    """Return a list of (kind, detail) violations; empty list = valid."""
    viols = []
    timelines, services = {}, {}
    for name, ele in robot_path.items():
        ev, sv = _timed_events(ele['path_cost'])
        if ev:
            timelines[name] = ev
            services[name] = sv

    # --- motion: pairwise continuous separation ---
    horizon = max((ev[-1][1] for ev in timelines.values()), default=0.0)
    names = list(timelines)
    for i in range(len(names) - 1):
        for j in range(i + 1, len(names)):
            r1 = robots_map[names[i]].robot_radius
            r2 = robots_map[names[j]].robot_radius
            d, at = _pair_min_distance(timelines[names[i]],
                                       timelines[names[j]], horizon)
            if d < r1 + r2 - _EPS_D:
                viols.append(('ROBOT_COLLISION',
                              '%s-%s min %.4f < %.4f at t=%.2f'
                              % (names[i], names[j], d, r1 + r2, at)))

    # --- tasks ---
    robots = {r.ind: r for r in obj_ins.get('robot', [])}
    jobs = {jb.ind: jb for jb in obj_ins.get('job', [])}
    required = set()
    for task in obj_ins.get('task', []):
        for goal in task.loc:
            required.add((goal, task.ind))

    goal_locs = {}
    wp_loc = {w.ind: w.loc for w in obj_ins.get('waypoint', [])}
    for jb in jobs.values():
        goal_locs[jb.ind] = [wp_loc[w] for w in jb.located if w in wp_loc]

    served = {}
    per_job = {}
    for name, sv in services.items():
        robot = robots.get(name)
        for (t0, t1, task, job, pos) in sv:
            served[(job, task)] = served.get((job, task), 0) + 1
            per_job.setdefault(job, []).append((t0, t1, task, name))
            locs = goal_locs.get(job)
            if locs and all(_dist(pos, lp) > 1e-3 for lp in locs):
                viols.append(('SERVICE_LOCATION',
                              '%s served %s/%s at %s, goal at %s'
                              % (name, job, task, pos, locs[0])))
            if robot is not None:
                dur = robot.action_duration.get(task)
                if dur is None:
                    viols.append(('CAPABILITY',
                                  '%s served %s without capability'
                                  % (name, task)))
                elif t1 - t0 < dur - _EPS_T:
                    viols.append(('SERVICE_DURATION',
                                  '%s %s %.2f < required %.2f'
                                  % (name, task, t1 - t0, dur)))

    for pair in required:
        n = served.get(pair, 0)
        if n == 0:
            viols.append(('TASK_MISSING', '%s/%s never served' % pair))
        elif n > 1:
            viols.append(('TASK_DUPLICATED', '%s/%s served %d times'
                          % (pair[0], pair[1], n)))

    for job, entries in per_job.items():
        entries.sort()
        for (a0, a1, ta, _na), (b0, b1, tb, _nb) in zip(entries, entries[1:]):
            if b0 < a1 - _EPS_T:
                viols.append(('STATION_RESOURCE',
                              '%s: %s and %s overlap' % (job, ta, tb)))
        ends = {task: t1 for (t0, t1, task, _n) in entries}
        starts_ = {task: t0 for (t0, t1, task, _n) in entries}
        jb = jobs.get(job)
        if jb is None:
            continue
        for order in jb.order:
            a, b = order[0], order[1]
            if a in ends and b in starts_ and starts_[b] < ends[a] - _EPS_T:
                viols.append(('PRECEDENCE',
                              '%s: %s starts %.2f before %s ends %.2f'
                              % (job, b, starts_[b], a, ends[a])))
    return viols
