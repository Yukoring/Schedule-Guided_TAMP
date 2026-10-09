"""Prioritized motion planning with SIPP on the shared roadmap.

Robots are planned sequentially in input order; each planned trajectory
(piecewise-linear, unit speed, parked forever at its final location)
becomes a dynamic obstacle for later robots. Each navigation leg is a
Safe Interval Path Planning search (Phillips & Likhachev 2011): every
roadmap vertex carries the time intervals during which it is clear of all
higher-priority robots, a state is (vertex, interval), and waiting is the
choice of a departure time within the interval - so waits, service stops
and permanent parking are handled exactly, in continuous time.

Collision semantics match the shared checker (motion/collision_vor.py):
two robots conflict when their center distance drops below the sum of
their radii; moving-vs-moving segments are resolved in closed form from
the quadratic of the linear relative motion.
"""
import heapq
import math

INF = float('inf')
NODE_CAP = 250000   # paper setting: prioritized search restricted to 250k nodes


def _lerp(a, b, ta, tb, t):
    # Static pieces (parking, vertex queries) span to INF; interpolating
    # them would produce INF/INF = NaN, which poisons every comparison.
    if a == b or tb <= ta:
        return a
    u = (t - ta) / (tb - ta)
    return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)


def _conflict_window(p0, p1, t0, t1, q0, q1, s0, s1, clearance):
    """Time window within [max(t0,s0), min(t1,s1)] where two linearly moving
 points come closer than clearance. None if they never do."""
    lo = max(t0, s0)
    hi = min(t1, s1)
    if lo >= hi:
        return None
    pa = _lerp(p0, p1, t0, t1, lo)
    pb = _lerp(p0, p1, t0, t1, hi)
    qa = _lerp(q0, q1, s0, s1, lo)
    qb = _lerp(q0, q1, s0, s1, hi)
    dx0, dy0 = pa[0] - qa[0], pa[1] - qa[1]
    dx1, dy1 = pb[0] - qb[0], pb[1] - qb[1]
    T = hi - lo
    vx, vy = (dx1 - dx0) / T, (dy1 - dy0) / T
    a = vx * vx + vy * vy
    b = 2 * (dx0 * vx + dy0 * vy)
    c = dx0 * dx0 + dy0 * dy0 - clearance * clearance
    if a < 1e-12:
        return (lo, hi) if c < 0 else None
    disc = b * b - 4 * a * c
    if disc <= 0:
        return None
    r = math.sqrt(disc)
    w_lo = max(0.0, (-b - r) / (2 * a))
    w_hi = min(T, (-b + r) / (2 * a))
    if w_lo >= w_hi:
        return None
    return (lo + w_lo, lo + w_hi)


class Trajectory:
    """Piecewise-linear trajectory; parked at its last location forever."""

    def __init__(self, breakpoints):
        self.pts = breakpoints            # [(loc, t), ...] with t non-decreasing
        self.end_time = breakpoints[-1][1]
        self.end_loc = breakpoints[-1][0]

    def pieces(self):
        for i in range(len(self.pts) - 1):
            (a, ta), (b, tb) = self.pts[i], self.pts[i + 1]
            if tb > ta:
                yield a, b, ta, tb
        yield self.end_loc, self.end_loc, self.end_time, INF   # parked


def _merge_blocked(windows):
    windows.sort()
    merged = []
    for lo, hi in windows:
        if merged and lo <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    return merged


def _safe_intervals(loc, trajectories, clearance):
    """Clear time intervals of a vertex against all planned trajectories."""
    blocked = []
    for traj in trajectories:
        for a, b, ta, tb in traj.pieces():
            w = _conflict_window(loc, loc, 0.0, INF, a, b, ta, tb, clearance)
            if w:
                blocked.append(list(w))
    if not blocked:
        return [(0.0, INF)]
    merged = _merge_blocked(blocked)
    safe = []
    t = 0.0
    for lo, hi in merged:
        if lo > t:
            safe.append((t, lo))
        t = max(t, hi)
    if t < INF:
        safe.append((t, INF))
    return safe


def _edge_clear(p0, p1, td, ta, trajectories, clearance):
    for traj in trajectories:
        for a, b, sa, sb in traj.pieces():
            if sb <= td or sa >= ta:
                continue
            if _conflict_window(p0, p1, td, ta, a, b, sa, sb, clearance):
                return False
    return True


def _edge_first_block_end(p0, p1, td, ta, trajectories, clearance):
    """Earliest end time among the pieces that block this edge departure, or None if the edge is clear."""
    first = None
    for traj in trajectories:
        for a, b, sa, sb in traj.pieces():
            if sb <= td or sa >= ta:
                continue
            if _conflict_window(p0, p1, td, ta, a, b, sa, sb, clearance):
                if first is None or sb < first:
                    first = sb
    return first


def _dijkstra_h(roadmap, samples, goal_idx):
    dist = {goal_idx: 0.0}
    heap = [(0.0, goal_idx)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, INF):
            continue
        for v in roadmap[u]:
            nd = d + math.dist(samples[u], samples[v])
            if nd < dist.get(v, INF):
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return dist


class _SippCounter:
    def __init__(self):
        self.expanded = 0


def _sipp(roadmap, samples, start_idx, goal_idx, start_time, hold,
          trajectories, clearance, counter):
    """Earliest-arrival SIPP leg. hold = time the robot must remain at the
 goal after arriving (service duration; INF demands safe-forever parking).
 @Return list of (u_idx, depart, v_idx, arrive) moves, or None."""
    iv_cache = {}

    def intervals(v):
        if v not in iv_cache:
            iv_cache[v] = _safe_intervals(samples[v], trajectories, clearance)
        return iv_cache[v]

    def interval_of(v, t):
        for k, (lo, hi) in enumerate(intervals(v)):
            if lo <= t + 1e-9 and t < hi + 1e-9:
                return k
        return None

    h = _dijkstra_h(roadmap, samples, goal_idx)
    k0 = interval_of(start_idx, start_time)
    if k0 is None or h.get(start_idx) is None:
        return None

    best = {(start_idx, k0): start_time}
    parent = {(start_idx, k0): None}
    heap = [(start_time + h[start_idx], start_time, start_idx, k0)]

    while heap:
        f, t, u, ku = heapq.heappop(heap)
        if t > best.get((u, ku), INF) + 1e-9:
            continue
        counter.expanded += 1
        if counter.expanded > NODE_CAP:
            return None
        iu_lo, iu_hi = intervals(u)[ku]

        if u == goal_idx:
            need = iu_hi if hold == INF else t + hold
            if (hold == INF and iu_hi == INF) or (hold != INF and need <= iu_hi + 1e-9):
                moves = []
                key = (u, ku)
                while parent[key] is not None:
                    pkey, td, ta = parent[key]
                    moves.append((pkey[0], td, key[0], ta))
                    key = pkey
                moves.reverse()
                return moves

        for v in roadmap[u]:
            tau = math.dist(samples[u], samples[v])
            if tau <= 0:
                continue
            for kv, (iv_lo, iv_hi) in enumerate(intervals(v)):
                ta = max(t + tau, iv_lo)
                td = ta - tau
                # an edge blocked at the earliest departure may be clear after a wait in the same safe interval:
                # scan candidate departures (0.1 s grid plus the end times of blocking pieces), earliest clear wins
                td0 = td
                hi = min(iu_hi, iv_hi - tau)
                feasible = False
                if td0 <= hi + 1e-9:
                    # 0.1 s departure grid; beyond the last piece end the world is static
                    last_end = td0
                    for traj in trajectories:
                        for _pa, _pb, _sa, sb in traj.pieces():
                            if sb != INF and sb > last_end:
                                last_end = sb
                    bound = min(hi, last_end + 0.1)
                    cands = {td0}
                    k = 1
                    while True:
                        c = td0 + 0.1 * k
                        if c > bound + 1e-9:
                            break
                        if k > 200000:
                            # search limit, not window coverage: record it
                            counter.grid_capped = getattr(
                                counter, 'grid_capped', 0) + 1
                            break
                        cands.add(c)
                        k += 1
                    for traj in trajectories:
                        for _pa, _pb, _sa, sb in traj.pieces():
                            if sb != INF and td0 < sb <= hi:
                                cands.add(sb + 1e-6)
                    if hi != INF:
                        cands.add(hi)
                    for td in sorted(cands):
                        ta = td + tau
                        if td > iu_hi + 1e-9 or ta > iv_hi + 1e-9:
                            continue
                        if _edge_clear(samples[u], samples[v], td, ta,
                                       trajectories, clearance):
                            feasible = True
                            break
                if not feasible:
                    continue
                if ta >= best.get((v, kv), INF) - 1e-9:
                    continue
                best[(v, kv)] = ta
                parent[(v, kv)] = ((u, ku), td, ta)
                heapq.heappush(heap, (ta + h.get(v, INF), ta, v, kv))
    return None


class PrioritizedPlanningSolver(object):
    """Sequential SIPP over the task plan's navigation/service legs.

 parsed_plan: {robot: [[start, goal_loc], [goal_loc, dur, task, goal], ...]}
 (same structure the PDDL generator's parse_plan produces).
 """

    def __init__(self, roadmap, samples, parsed_plan, rr=0.3, starts=None):
        # `starts` maps robot name -> start location; a robot without any action stays there for the whole horizon
        # and is a stationary obstacle for lower-priority robots
        self.roadmap = roadmap
        self.samples = samples
        self.clearance = rr * 2 * 1.1
        self.robot_paths = dict()
        self.legs = []       # per robot: list of ('move', loc) / ('task', dur, task, goal)
        self.starts = []
        self.names = []
        self.idle = set()
        for name, ele in parsed_plan.items():
            legs = []
            start = None
            if not ele and starts is not None and name in starts:
                start = starts[name]
                self.idle.add(name)
            for way in ele:
                if start is None:
                    start = way[0]
                if type(way[1]) == tuple:
                    legs.append(('move', way[1]))
                else:
                    legs.append(('task', way[1], way[2], way[3]))
            self.names.append(name)
            self.starts.append(start)
            self.legs.append(legs)
            self.robot_paths[name] = dict()
        self.CPU_time = 0

    def find_solution(self):
        import time as timer
        t0 = timer.time()
        trajectories = []
        counter = _SippCounter()

        for i, name in enumerate(self.names):
            if name in self.idle:
                # idle robot: stationary at its start for the whole horizon
                breakpoints = [(self.starts[i], 0.0)]
                trajectories.append(Trajectory(breakpoints))
                self.robot_paths[name]['mission_end'] = 0.0
                self.robot_paths[name]['breakpoints'] = breakpoints
                continue
            idx = self.samples.index(self.starts[i])
            t = 0.0
            breakpoints = [(self.starts[i], 0.0)]
            events = []          # (t_start, dur, task, goal) service stops
            legs = self.legs[i]
            for j, leg in enumerate(legs):
                if leg[0] == 'task':
                    events.append((t, leg[1], leg[2], leg[3]))
                    t += leg[1]
                    breakpoints.append((breakpoints[-1][0], t))
                    continue
                goal_loc = leg[1]
                goal_idx = self.samples.index(goal_loc)
                # required stay after arrival: following service, or forever
                # if this is the robot's last navigation leg
                hold = 0.0
                if j + 1 < len(legs) and legs[j + 1][0] == 'task':
                    hold = legs[j + 1][1]
                if all(l[0] != 'move' for l in legs[j + 1:]):
                    hold = INF
                moves = _sipp(self.roadmap, self.samples, idx, goal_idx, t,
                              hold, trajectories, self.clearance, counter)
                if moves is None:
                    print('no path (agent %s leg %d, %d states)'
                          % (name, j, counter.expanded))
                    return None, -1, -1
                for u, td, v, ta in moves:
                    if td > t + 1e-9:
                        breakpoints.append((self.samples[u], td))   # wait
                    breakpoints.append((self.samples[v], ta))
                    t = ta
                idx = goal_idx
            mission_end = t if not events or events[-1][0] + events[-1][1] <= t \
                else events[-1][0] + events[-1][1]

            # the robot parks where its plan ends
            trajectories.append(Trajectory(breakpoints))
            self._store(name, breakpoints, events)
            self.robot_paths[name]['mission_end'] = mission_end
            self.robot_paths[name]['breakpoints'] = breakpoints

        # makespan counts mission completion; the trajectory (incl. retreat)
        # is what the shared collision checker sweeps.
        makespan = max(ele['mission_end'] for ele in self.robot_paths.values())
        total_cost = sum(ele['mission_end'] for ele in self.robot_paths.values())
        # idle robots: single-node path for the whole horizon
        if self.idle:
            horizon = max((bp[-1][1] for n, ele in self.robot_paths.items()
                           if n not in self.idle for bp in [ele['breakpoints']]), default=0.0)
            for name in self.idle:
                loc = self.starts[self.names.index(name)]
                self.robot_paths[name]['path_cost'] = [[loc, horizon, [(horizon, 2)]]]
                self.robot_paths[name]['path_only'] = [loc]
                self.robot_paths[name]['path_time'] = horizon
        self.CPU_time = timer.time() - t0
        print('Prioritized SIPP: %d states expanded, %.2fs'
              % (counter.expanded, self.CPU_time))
        return self.robot_paths, makespan, total_cost

    def _store(self, name, breakpoints, events):
        """Convert breakpoints + service events into the legacy path format
 ('path_cost' with [loc, stop_dur, markers] stop entries).

 Invariants enforced: a stop always extends the element the robot is
 currently at, and the reconstructed duration of the emitted structure
 matches the trajectory's end time (the shared collision checker
 rebuilds timing from path_cost, so any mismatch would silently shift
 the whole trajectory).
 """
        path_cost = [breakpoints[0][0]]
        ev = list(events)

        for k in range(1, len(breakpoints)):
            (a, ta), (b, tb) = breakpoints[k - 1], breakpoints[k]
            if a == b:
                if tb - ta <= 1e-9:
                    continue
                markers = []
                t = ta
                while ev and ev[0][0] < tb - 1e-6:
                    _, dur, task, goal = ev.pop(0)
                    markers.append((dur, task, goal))
                    t += dur
                if tb - t > 1e-6:
                    markers.append((tb - t, 2))
                last = path_cost[-1]
                if isinstance(last, list):
                    assert last[0] == a, (name, last[0], a)
                    last[1] += tb - ta
                    last[2] += markers
                else:
                    assert last == a, (name, last, a)
                    path_cost[-1] = [a, tb - ta, markers]
            else:
                path_cost.append(b)

        # verify the legacy reconstruction reproduces the trajectory timing
        t = 0.0
        prev = None
        for ele in path_cost:
            loc = ele[0] if isinstance(ele, list) else ele
            if prev is not None:
                t += math.dist(prev, loc)
            if isinstance(ele, list):
                t += ele[1]
            prev = loc
        end = breakpoints[-1][1]
        assert abs(t - end) < 0.5 + 0.01 * len(path_cost), \
            ('%s: reconstructed %.2f vs trajectory %.2f' % (name, t, end))

        path_only = []
        for ele in path_cost:
            loc = ele[0] if isinstance(ele, list) else ele
            if not path_only or path_only[-1] != loc:
                path_only.append(loc)

        self.robot_paths[name]['path_cost'] = path_cost
        self.robot_paths[name]['path_only'] = path_only
        self.robot_paths[name]['path_time'] = end
