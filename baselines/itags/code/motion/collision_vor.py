from copy import deepcopy
import enum
import math
import random
import copy
import time
from shapely.geometry import Point, LineString
from shapely import ops
from common.utils import *

def collision_check(robot_path, robots, prm, col_env, transient_mem=None,
                    it_count=0):
    """
    Collision Check Function
    Robot path, robots: dictionary

    transient_mem (optional, mutable list) enables the transient tier for
    open-space crossing conflicts: a normal-type collision seen for the
    first time is remembered ({'center','samples','iter'}) and withheld
    from region carving - the caller feeds its samples to the scheduling
    relaxation instead, which separates the crossing times. Only when a
    later iteration collides near a remembered center is the conflict
    treated as structural and carved as usual (escalation).
    """
    collision = []
    key_list = list(robot_path)
    robot_num = len(key_list)
    collision_time = time.time()
    for i in range(robot_num - 1):
        for j in range(i+1, robot_num):
            robot1_name = key_list[i]
            robot2_name = key_list[j]
            robot1_path = robot_path[robot1_name]
            robot2_path = robot_path[robot2_name]
            robot1_rr = robots[robot1_name].robot_radius
            robot2_rr = robots[robot2_name].robot_radius
            collision = collision + make_collision_line(prm, robot1_path, robot1_rr, robot2_path, robot2_rr)
    collision_elapsed_time = time.time() - collision_time
    if prm.env.debug_mode:
        print("Collision Checking Time: ", collision_elapsed_time)
    # Late Token case
    if col_env == -1:
        return collision, -1
    narrow_col = []
    normal_col = []
    pick_collision = None
    geo_of = {}
    for ele in collision:
        geo = ele.pop(1)   # entry becomes [samples] again for downstream
        geo_of[id(ele)] = geo
        if geo['narrow']:
            ele.append(0)
            narrow_col.append(ele)
        else:
            ele.append(1)
            normal_col.append(ele)

    if transient_mem is not None and normal_col:
        thresh = prm.max_rr * 2
        escalated = []
        for ele in normal_col:
            samples = ele[0]
            if not samples:
                # degenerate report (no samples): leave on the carve path
                escalated.append(ele)
                continue
            cx = sum(s[0] for s in samples) / len(samples)
            cy = sum(s[1] for s in samples) / len(samples)
            hit = None
            for m in transient_mem:
                if math.hypot(cx - m['center'][0], cy - m['center'][1]) < thresh:
                    hit = m
                    break
            if hit is None:
                transient_mem.append({'center': (cx, cy),
                                      'samples': list(samples),
                                      'iter': it_count})
            elif hit['iter'] == it_count:
                pass  # sibling variant re-reporting the same crossing
            else:
                # recurred in a later iteration: structural after all
                escalated.append(ele)
                # Only one conflict is carved below. Keep every pending
                # conflict until its samples are actually covered.
        normal_col = escalated

    # Randomly Choose one collision
    if narrow_col:
        n = len(narrow_col)
        pick_collision = narrow_col[random.randint(0,n-1)]
    elif normal_col:
        n = len(normal_col)
        pick_collision = normal_col[random.randint(0,n-1)]

    if prm.env.debug_mode:
        print("Pick One Collision", pick_collision, len(collision))
    unity_col = copy.deepcopy(col_env)
    if pick_collision is not None:
        unity_col = handle_collision(prm, pick_collision, col_env,
                                     conflict=geo_of.get(id(pick_collision)))
        if transient_mem is not None:
            covered = {tuple(s) for region in unity_col for s in region[1]}
            transient_mem[:] = [
                m for m in transient_mem
                if not m['samples'] or
                not {tuple(s) for s in m['samples']}.issubset(covered)
            ]
    #print(unity_col)

    return collision, unity_col

def handle_collision(prm, collision, col_env, conflict=None):
    """Fold one picked conflict into the region set - canonically.

    Every region remembers the raw conflicts that created it
    (meta = entry[3] = {'conflicts': [{'pts','narrow','throat'}...],
    'throat', 'area'}), and its footprint is a pure function of that set:
    the union of the conflict points buffered by 2*max_rr. Merging is
    "connected components of overlapping footprints", so the resulting
    regions are independent of the order in which conflicts were
    discovered - the old incremental buffer-union grew a different map
    per discovery order and could never shrink cleanly.

    Entry layout stays [entries, samples, type(, meta)] for downstream.
    """
    max_rr = prm.max_rr
    task_samples = prm.task_samples

    def footprint(confs):
        return ops.unary_union([Point(p).buffer(2 * max_rr)
                                for c in confs for p in c['pts']])

    def meta_of(entry):
        if len(entry) > 3 and entry[3].get('conflicts'):
            return entry[3]
        # legacy entry without provenance: treat its samples as the pts
        m = {'conflicts': [{'pts': list(entry[1]),
                            'narrow': entry[2] == 0,
                            'throat': None}],
             'throat': None}
        m['area'] = footprint(m['conflicts'])
        return m

    new_conf = dict(conflict) if conflict else {'pts': list(collision[0]),
                                                'narrow': collision[1] == 0,
                                                'throat': None}
    conflicts = [new_conf]
    area = footprint(conflicts)

    remaining = list(col_env)
    changed = True
    while changed:
        changed = False
        keep = []
        for entry in remaining:
            m = meta_of(entry)
            if 'area' not in m:
                m['area'] = footprint(m['conflicts'])
            if m['area'].intersects(area):
                conflicts += m['conflicts']
                area = area.union(m['area'])
                changed = True
            else:
                keep.append(entry)
        remaining = keep

    samples = [smp for smp in task_samples if area.covers(Point(smp))]
    col_type = 0 if any(c['narrow'] for c in conflicts) else 1
    throats = [c['throat'] for c in conflicts if c.get('throat') is not None]
    new_col = handle_endpoint(prm, [samples, col_type])
    new_col.append({'conflicts': conflicts,
                    'throat': min(throats) if throats else None,
                    'area': area})
    remaining.append(new_col)
    return remaining

def handle_endpoint(prm, new_col):
    new_entry = None
    max_rr = prm.max_rr
    task_samples = prm.task_samples

    # Make Roadmap without collision samples
    c_roadmap_samples = []
    c_roadmap = []
    for sample in task_samples:
        if not sample in new_col[0]:
            c_roadmap_samples.append(sample)

    # Separate internal waypoints and external waypoints
    important = []
    delete_vor_way = []
    new_vor_way = []
    external_imp = []
    internal_imp = []

    for robot, ele in prm.robots_map.items():
        important.append(ele.pos)   # Robot pos
    for ele in prm.goal_samples:
        important += ele            # Goal pos
    for ele in prm.vor_ways:
        if ele in new_col[0]:
            delete_vor_way.append(ele)
        else:
            external_imp.append(ele)    # Voronoi pos

    for ele in important:
        if ele in new_col[0]:
            internal_imp.append(ele)
        else:
            external_imp.append(ele)

    # 1. Make End Points
    """
    From the current start points, pick the two farthest points as end points and drop the points around them.
    """

    endpoint_time = time.time()
    col_samples = new_col[0]
    sample_num = len(col_samples)
    temp_end = []

    buffered_end = []
    delete_samples = []
    start_samples = []

    for ele in internal_imp:
        buffered_end.append(Point(ele).buffer(max_rr*3.0)) #VRP CHANGE 3.0 -> 4.0
        temp_end.append(ele)

    for sample in col_samples:
        p_sample = Point(sample)
        for ele in buffered_end:
            if ele.contains(p_sample):
                delete_samples.append(sample)
                break
    for sample in col_samples:
        if not sample in delete_samples:
            start_samples.append(sample)

    while True:
        if not start_samples:
            break
        sample_num = len(start_samples)
        if sample_num == 1:
            temp_end.append(start_samples[0])
            break
        else:
            a = None
            b = None
            max_dist = 0
            temp_start = []
            for i in range(sample_num-1):
                temp_a = start_samples[i]
                for j in range(i+1, sample_num):
                    temp_b = start_samples[j]
                    if get_distance(temp_a, temp_b) > max_dist:
                        max_dist = get_distance(temp_a, temp_b)
                        a = temp_a
                        b = temp_b
            if max_dist > max_rr * 4.0: # VRP 4.0 -> 6.0
                temp_end.append(a)
                temp_end.append(b)
                buffered_a = Point(a).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                buffered_b = Point(b).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                for sample in start_samples:
                    p_sample = Point(sample)
                    if not buffered_a.contains(p_sample) and not buffered_b.contains(p_sample):
                        temp_start.append(sample)
                start_samples = temp_start
            else:
                min_dist = 100000
                max_num_sam = -1
                end_candidate = None
                for ele1 in start_samples:
                    temp_num_sam = 0
                    buf_cand = Point(ele1).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                    for ele2 in c_roadmap_samples:
                        p_sam = Point(ele2)
                        if buf_cand.contains(p_sam):
                            temp_num_sam += 1
                    if max_num_sam < temp_num_sam:
                        max_num_sam = temp_num_sam
                        end_candidate = ele1

                # for ele1 in external_imp:
                #     for ele2 in start_samples:
                #         if get_distance(ele1, ele2) < min_dist:
                #             min_dist = get_distance(ele1, ele2)
                #             end_candidate = ele2
                if end_candidate:
                    temp_end.append(end_candidate)
                    buffered_candidate = Point(end_candidate).buffer(max_rr*3.0) # VRP 3.0-> 4.0
                    for sample in start_samples:
                        p_sample = Point(sample)
                        if not buffered_candidate.contains(p_sample):
                            temp_start.append(sample)
                    start_samples = temp_start
                else:
                    break

    # Voronoi waypoints
    for ele in temp_end:
        for ele2 in prm.vor_ways:
            if get_distance(ele, ele2) < max_rr*2.5:
                delete_vor_way.append(ele2)

    if prm.env.debug_mode:
        #print("End Point Candidates: ", temp_end)
        endpoint_elapsed_time = time.time() - endpoint_time
        print("Endpoint Making Time: ", endpoint_elapsed_time)

    # 2. Make entry points
    """
    One entry point may connect to several end points.
    """
    if new_col[1] == 1:
        col_end = []
        for ele in temp_end:
            col_end.append([ele])
        new_entry = [col_end, col_samples, new_col[1]]
    else:
        entry_time = time.time()
        #c_roadmap_samples, temp_end
        temp_nears = []
        near_samples = {}
        for sample in c_roadmap_samples:
            p_sample = Point(sample)
            near_flag = False
            for ele in temp_end:
                end_buf = Point(ele).buffer(max_rr*3.5)
                if end_buf.contains(p_sample):
                    near_flag = True
                    break
            if near_flag:
                temp_nears.append(sample)

        for sample in temp_nears:
            min_dist = 100000
            min_end = None
            for ele in temp_end:
                if get_distance(sample, ele) < min_dist:
                    min_dist = get_distance(sample,ele)
                    min_end = ele
            if min_dist > max_rr *2 or (sample in external_imp and not sample in prm.vor_ways):
                near_samples[min_end] = near_samples.get(min_end, []) + [sample]

        # print(near_samples, len(near_samples))

        temp_entry = []
        entry_dict = {}

        for ele in temp_end:
            entry_candidates = copy.deepcopy(near_samples.get(ele, []))
            temp_external = []
            entry_dict[ele] = []
            # For important way as a entry
            for ele2 in entry_candidates:
                if ele2 in external_imp and not ele2 in prm.vor_ways:
                    temp_entry.append(ele2)
                    temp_external.append(ele2)
                    entry_dict[ele] = entry_dict.get(ele, []) + [ele2]
            temp_entry_candidates = []
            for ele2 in entry_candidates:
                p = Point(ele2)
                imp_flag = False
                for ele3 in temp_external:
                    if Point(ele3).buffer(max_rr*3.0).contains(p):
                        imp_flag = True
                        break
                if not imp_flag:
                    temp_entry_candidates.append(ele2)
            entry_candidates = temp_entry_candidates

            while True:
                candi_len = len(entry_candidates)
                if candi_len == 0:
                    break
                elif candi_len == 1:
                    temp_entry.append(entry_candidates[0])
                    entry_dict[ele] = entry_dict.get(ele, []) + [entry_candidates[0]]
                    break
                else:
                    a = None
                    b = None
                    max_dist = 0
                    temp_start = []
                    for i in range(candi_len-1):
                        temp_a = entry_candidates[i]
                        for j in range(i+1, candi_len):
                            temp_b = entry_candidates[j]
                            if get_distance(temp_a, temp_b) > max_dist:
                                max_dist = get_distance(temp_a, temp_b)
                                a = temp_a
                                b = temp_b
                    if max_dist > max_rr * 2.5:
                        temp_entry.append(a)
                        temp_entry.append(b)
                        entry_dict[ele] = entry_dict.get(ele, []) + [a]
                        entry_dict[ele] = entry_dict.get(ele, []) + [b]
                        buffered_a = Point(a).buffer(max_rr*3.0)
                        buffered_b = Point(b).buffer(max_rr*3.0)
                        for sample in entry_candidates:
                            p_sample = Point(sample)
                            if not buffered_a.contains(p_sample) and not buffered_b.contains(p_sample):
                                temp_start.append(sample)
                        entry_candidates = temp_start
                    else:
                        min_dist = 100000
                        one_entry = None
                        for ele2 in entry_candidates:
                            if get_distance(ele, ele2) < min_dist:
                                min_dist = get_distance(ele, ele2)
                                one_entry = ele2
                        if one_entry:
                            temp_entry.append(one_entry)
                            entry_dict[ele] = entry_dict.get(ele, []) + [one_entry]
                            buffered_candidate = Point(one_entry).buffer(max_rr*3.0)
                            for sample in entry_candidates:
                                p_sample = Point(sample)
                                if not buffered_candidate.contains(p_sample):
                                    temp_start.append(sample)
                            entry_candidates = temp_start
                        else:
                            break

        # Vornoi delete
        for ele in temp_entry:
            for ele2 in prm.vor_ways:
                if get_distance(ele, ele2) < max_rr*2.5:
                    delete_vor_way.append(ele2)

        find_entry = []
        for name, description in entry_dict.items():
            temp_find = [name]
            temp_find += description
            find_entry.append(temp_find)

        new_entry = [find_entry, new_col[0], new_col[1]]

    # Setting The Voronoi Sample for the save
    for ele in prm.vor_ways:
        if not ele in delete_vor_way:
            new_vor_way.append(ele)

    prm.vor_ways = new_vor_way

    return new_entry

def make_collision_line(prm, path1, rr1, path2, rr2):
    """
    Waiting Timing must be added later.
    """
    path1_cost = path1['path_cost']
    path2_cost = path2['path_cost']
    max_time = int(max(path1['path_time'], path2['path_time']) * 10)

    # agent 1
    path_with_time1 = []
    temp_time = 0.0
    path_len = len(path1_cost)
    starts = None
    if path_len > 1:
        for i in range(1, path_len):
            if i == 1:
                if type(path1_cost[i-1]) == list:
                    for partial in path1_cost[i-1][2]:
                        if partial[1] == 2:
                            path_with_time1.append((path1_cost[i-1][0], temp_time))
                            temp_time += round(partial[0], 2)
                        else:
                            path_with_time1.append((path1_cost[i-1][0], temp_time, partial[1], partial[2]))
                            temp_time += round(partial[0], 2)
                    path_with_time1.append((path1_cost[i-1][0], temp_time))
                    starts = path1_cost[i-1][0]
                else:
                    path_with_time1.append((path1_cost[i-1], temp_time))
                    starts = path1_cost[i-1]
            if type(path1_cost[i]) == list:
                temp_time += round(get_distance(starts, path1_cost[i][0]), 2)
                path_with_time1.append((path1_cost[i][0], temp_time))
                for partial in path1_cost[i][2]:
                    if partial[1] == 2:
                        temp_time += round(partial[0], 2)
                        path_with_time1.append((path1_cost[i][0], temp_time))
                    else:
                        temp_time += round(partial[0], 2)
                        path_with_time1.append((path1_cost[i][0], temp_time, partial[1], partial[2]))
                starts = path1_cost[i][0]
            else:
                temp_time += round(get_distance(starts, path1_cost[i]), 2)
                path_with_time1.append((path1_cost[i], temp_time))
                starts = path1_cost[i]
    else:
        path_with_time1 = [(path1_cost[0][0], temp_time)]

    # agent 2
    path_with_time2 = []
    temp_time = 0.0
    path_len = len(path2_cost)
    starts = None
    if path_len > 1:
        for i in range(1, path_len):
            if i == 1:
                if type(path2_cost[i-1]) == list:
                    for partial in path2_cost[i-1][2]:
                        if partial[1] == 2:
                            path_with_time2.append((path2_cost[i-1][0], temp_time))
                            temp_time += round(partial[0], 2)
                        else:
                            path_with_time2.append((path2_cost[i-1][0], temp_time, partial[1], partial[2]))
                            temp_time += round(partial[0], 2)
                    path_with_time2.append((path2_cost[i-1][0], temp_time))
                    starts = path2_cost[i-1][0]
                else:
                    path_with_time2.append((path2_cost[i-1], temp_time))
                    starts = path2_cost[i-1]
            if type(path2_cost[i]) == list:
                temp_time += round(get_distance(starts, path2_cost[i][0]), 2)
                path_with_time2.append((path2_cost[i][0], temp_time))
                for partial in path2_cost[i][2]:
                    if partial[1] == 2:
                        temp_time += round(partial[0], 2)
                        path_with_time2.append((path2_cost[i][0], temp_time))
                    else:
                        temp_time += round(partial[0], 2)
                        path_with_time2.append((path2_cost[i][0], temp_time, partial[1], partial[2]))
                starts = path2_cost[i][0]
            else:
                temp_time += round(get_distance(starts, path2_cost[i]), 2)
                path_with_time2.append((path2_cost[i], temp_time))
                starts = path2_cost[i]
    else:
        path_with_time2 = [(path2_cost[0][0], temp_time)]

    collision = []

    temp_col = []
    prev_col = []

    timestep = 0
    for timestep in range(0, max_time):
        pos1 = get_state_time(timestep / 10, path_with_time1)
        pos2 = get_state_time(timestep / 10, path_with_time2)
        if get_distance(pos1, pos2) < rr1 + rr2:
            prev_col.append(pos1)
            prev_col.append(pos2)
        else:
            if prev_col:
                temp_col.append(prev_col)
                prev_col = []
    if prev_col:
        # a collision persisting through the final timestep is reported too
        temp_col.append(prev_col)

    samples = prm.task_samples
    max_rr = prm.max_rr

    for ele in temp_col:
        collision_sample = []
        for sample in samples:
            for pos in ele:
                if get_distance(pos, sample) < max_rr*2.0:
                    collision_sample.append(sample)
                    break
        # narrow-passage conflict: two discs cannot pass abreast at the widest point of the conflict
        throat = max(prm.env.clearance(p) for p in ele)
        narrow = throat < (rr1 + rr2) + 0.1 * max_rr
        collision.append([collision_sample, {'narrow': narrow,
                                             'throat': throat,
                                             'pts': list(ele)}])

    return collision

def get_state_time(t, path):
    if t <= path[0][1]:
        return path[0][0]
    elif t >= path[-1][1]:
        return path[-1][0]
    else:
        # times are non-decreasing: locate the last entry with time <= t
        lo, hi = 0, len(path) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if path[mid][1] <= t:
                lo = mid
            else:
                hi = mid
        i = lo
        x1, y1 = path[i][0]
        x2, y2 = path[i+1][0]
        dx = x2 - x1
        dy = y2 - y1
        yaw = math.atan2(dy, dx)

        if dx ==0 and dy == 0:
            return (x2, y2)

        pos_x = x1 + (t - path[i][1]) * 1 * math.cos(yaw)
        pos_y = y1 + (t - path[i][1]) * 1 * math.sin(yaw)
        pos = (pos_x, pos_y)

        return pos
