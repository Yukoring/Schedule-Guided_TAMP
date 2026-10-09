import copy
import math
import time
from math import degrees
from pathlib import Path

import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString

from common.utils import get_sum_of_cost, get_path, KDTree, get_distance
from common.visualize import *


class BasePDDLGenerator:
    """Shared skeleton of every PDDL problem generator.

    Holds the environment instantiation (obj_ins), collision-driven
    environment updates, OPTIC plan parsing and the mapping from a parsed
    plan to robot trajectories. Subclasses provide the problem encodings
    (generate_initial / generate_goals / generate_metric / generate_problem).
    """
    def __init__(self, env, prm, domain, pFile):
        self.env = env
        self.robots = env.robots_map
        self.goals = env.goals_map
        self.prm = prm
        self.samples = prm.samples
        self.roadmaps = prm.roadmaps
        self.task_samples = prm.task_samples
        self.task_roadmap = prm.task_roadmap
        self.task_rr = prm.max_rr

        self.pFilename = pFile
        self.pFile = None
        self.domain = domain

        self.obj_ins = dict()
        self.basic_way_num = 0

        self.velocity = 1.0

    class Instance:
        def __init__(self, ind, loc, instype):
          self.ind = ind
          self.loc = loc
          self.instype = instype

          # Waypoint Var
          self.connected = []
          self.distance = []
          self.path = []
          self.vertex = True
          self.collision_connect = []
          self.reserved = False
          self.token_ind = None
          self.sample_ind = None
          self.connect_ind = []

          # Job Var
          self.located = []
          self.order = []

          # Robot Var
          self.at = []
          self.action_duration = dict()
          self.radius = 0.0

          # Token Var
          self.token_name = ''
          self.token_wp = []
          self.token_samples = []

    def instance_init(self):
        self.obj_ins = dict()

    def _install_pots(self, wpind, k):
        """Staging waypoints around every goal sample so that a robot headed to a busy goal can wait next to it: 8 compass candidates per goal,
        the first k collision-free and clearance-separated ones are kept."""
        rr = self.task_rr
        ring = 2.6 * rr           # outside the service spot and its region
        sep = 2.4 * rr            # parked robots never touch anything
        existing = [w.loc for w in self.obj_ins['waypoint']]
        for job in self.obj_ins['job']:
            if not job.located:
                continue
            base = next(w.loc for w in self.obj_ins['waypoint']
                        if w.ind == job.located[0])
            placed = 0
            for gi in range(8):
                if placed >= k:
                    break
                ang = gi * math.pi / 4
                q = (round(base[0] + ring * math.cos(ang), 2),
                     round(base[1] + ring * math.sin(ang), 2))
                if not self.prm.isPointCollisionFree(q, rr) \
                        or self.prm.isOutOfBounds(q, rr):
                    continue
                if any(get_distance(q, p) < sep for p in existing):
                    continue
                pot = self.Instance('wp' + str(wpind), q, 'normal_way')
                wpind += 1
                self.obj_ins['waypoint'].append(pot)
                existing.append(q)
                placed += 1
        return wpind

    def env_instance(self, pots=0):
        # Robot
        wpind = 0
        for name, robot_map in self.robots.items():
            pos = robot_map.pos
            waypoint = self.Instance('wp'+str(wpind), pos, 'normal_way')
            waypoint.vertex = False
            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]

            robot = None
            robot = self.Instance(name, pos, 'robot')
            for i in range(len(robot_map.service)):
                robot.action_duration[robot_map.service[i]] = robot_map.action_cost[i]
            robot.at.append('wp'+str(wpind))
            robot.radius = robot_map.robot_radius
            self.obj_ins['robot'] = self.obj_ins.get('robot', []) + [robot]
            wpind += 1

        # Goal
        temp_task = dict()
        for name, goal_map in self.goals.items():
            job = self.Instance(name, goal_map['samples'], 'job')
            for i in range(len(goal_map['service'])):
                temp_task[(goal_map['service'][i], goal_map['service_type'][i])] = temp_task.get((goal_map['service'][i], goal_map['service_type'][i]), []) + [name]
            for sample in goal_map['samples']:
                waypoint = self.Instance('wp'+str(wpind), sample, 'normal_way')
                job.located.append('wp'+str(wpind))
                wpind += 1
                self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [waypoint]
            for ele in goal_map['order']:
                job.order.append(ele)
            self.obj_ins['job'] = self.obj_ins.get('job', []) + [job]

        if pots:
            # Before basic_way_num is fixed, so pots survive env_update's
            # truncation to the basic waypoint set.
            wpind = self._install_pots(wpind, pots)

        # every goal task must be served by some robot
        all_services = {s for r in self.obj_ins['robot'] for s in r.action_duration}
        for (task_name, _t) in temp_task:
            if task_name not in all_services:
                print('WARNING: no robot can perform task %r (robot services: %s)'
                      % (task_name, sorted(all_services)))

        for name, task_map in temp_task.items():
            name1, name2 = name
            task = self.Instance(name1, task_map, name2)
            self.obj_ins['task'] = self.obj_ins.get('task', []) + [task]
        self.basic_way_num = len(self.obj_ins['waypoint'])

        # Calculate Distance between waypoints
        # obj_ins maps a type to a list of instances

        way_list = self.obj_ins['waypoint']

        temp_samples = []
        remove_samples = []
        for way in way_list:
            way_buffer = Point(way.loc).buffer(self.task_rr)
            for w in self.task_samples:
                w_point = Point(w).buffer(self.task_rr)
                if way_buffer.intersects(w_point):
                    remove_samples.append(w)
        remove_samples = set(remove_samples)
        for w in self.task_samples:
            if not w in remove_samples:
                temp_samples.append(w)
        temp_samples = list(set(temp_samples))
        temp_skdtree = KDTree(temp_samples)
        temp_roadmap = []
        cd = self.prm.casting_dist[self.task_rr]
        nsample = len(temp_samples)
        knn = min(int(len(self.task_samples)/40), 15)
        for (ii, temp_point) in zip(range(nsample), temp_samples):
            # bounded KNN with the same widening rule as prm.generate_roadmap
            # (the old k=nsample sorted every sample per query and ran
            # millions of edge checks on large maps)
            for k in (min(nsample, 8 * knn + 1), nsample):
                index, dists = temp_skdtree.search(temp_point, k=k)
                edge_id = []
                for jj in range(1, len(index)):
                    npoint = temp_samples[index[jj]]
                    if self.prm.isEdgeCollisionFree(temp_point, npoint, self.task_rr) and \
                        get_distance(temp_point, npoint) < cd:
                        edge_id.append(index[jj])
                    if len(edge_id) >= knn:
                        break
                if len(edge_id) >= knn or k == nsample or dists[-1] >= cd:
                    break
            temp_roadmap.append(edge_id)


        # One single-source Dijkstra per waypoint instead of one search per
        # pair; a destination is closed off through its own attachment edges,
        # which reproduces the old pairwise-augmented query results.
        n_way = len(way_list)
        attach = [self.prm.attach_point(w.loc, temp_samples, temp_skdtree, self.task_rr)
                  for w in way_list]
        settled = [self.prm.dijkstra_all(w.loc, attach[i], temp_roadmap, temp_samples)
                   for i, w in enumerate(way_list)]

        def route(src, dst):
            goal_loc = way_list[dst].loc
            best_node, best_cost = None, None
            for k in attach[dst]:
                node = settled[src].get(k)
                if node is None:
                    continue
                c = node['cost'] + math.hypot(goal_loc[0] - node['loc'][0],
                                              goal_loc[1] - node['loc'][1])
                if best_cost is None or c < best_cost:
                    best_cost, best_node = c, node
            if best_node is None:
                return None
            return get_path(best_node) + [goal_loc]

        for i in range(n_way - 1):
            for j in range(i + 1, n_way):
                start_pos = way_list[i].loc
                goal_pos = way_list[j].loc
                if start_pos == goal_pos:
                    path1 = [start_pos, goal_pos]
                    path2 = [goal_pos, start_pos]
                elif get_distance(start_pos, goal_pos) < self.task_rr * 5 and \
                        self.prm.isEdgeCollisionFree(start_pos, goal_pos, self.task_rr):
                    path1, cost1 = self.prm.QueryTaskPlan(start_pos, goal_pos)
                    path2, cost2 = self.prm.QueryTaskPlan(goal_pos, start_pos)
                    if cost1 == -1:
                        path1 = [start_pos, goal_pos]
                    if cost2 == -1:
                        path2 = [goal_pos, start_pos]
                else:
                    path1 = route(i, j)
                    path2 = route(j, i)
                if getattr(self.prm, 'smooth', False):
                    path1 = self.prm.prune(path1, self.task_rr)
                    path2 = self.prm.prune(path2, self.task_rr)

                if path1 is None or path2 is None:
                    temp_dist1 = -1
                    temp_dist2 = -1
                else:
                    temp_dist1 = round(get_sum_of_cost(path1), 2)
                    temp_dist2 = round(get_sum_of_cost(path2), 2)
                    if temp_dist1 < temp_dist2:
                        path2 = copy.deepcopy(path1)
                        path2.reverse()
                        temp_dist2 = round(get_sum_of_cost(path2), 2)
                    else:
                        path1 = copy.deepcopy(path2)
                        path1.reverse()
                        temp_dist1 = round(get_sum_of_cost(path1), 2)

                if temp_dist1 != -1:
                    way_list[i].distance.append(temp_dist1)
                    way_list[i].connected.append(way_list[j].ind)
                    way_list[i].path.append(path1)
                if temp_dist2 != -1:
                    way_list[j].distance.append(temp_dist2)
                    way_list[j].connected.append(way_list[i].ind)
                    way_list[j].path.append(path2)

        # The task-level attachment can fail for a goal the PRM-level check
        # reached (different attachment mechanics); repair here too.
        self._reconnect_unreachable()

    def _reconnect_unreachable(self):
        """Completeness guard: reconnect basic waypoints that are cut off
        from the robot starts in the task abstraction, via the full
        roadmap. A stranded goal waypoint makes every encoding unsolvable
        (OPTIC: 'goal fact cannot be found'). Applies to normal_way and to
        basic waypoints absorbed into collision regions."""
        way_list = self.obj_ins['waypoint']
        way_by_ind = {w.ind: w for w in way_list}
        start_inds = [way_list[i].ind for i in range(len(self.robots))]

        def _reachable():
            seen = set(start_inds)
            stack = list(start_inds)
            while stack:
                for nxt in way_by_ind[stack.pop()].connected:
                    if nxt not in seen and nxt in way_by_ind:
                        seen.add(nxt)
                        stack.append(nxt)
            return seen

        for _ in range(self.basic_way_num):
            seen = _reachable()
            missing = [way_list[i] for i in range(self.basic_way_num)
                       if way_list[i].ind not in seen
                       and way_list[i].instype in ('normal_way', 'col_way',
                                                   'reserved_way')]
            if not missing:
                break
            wp1 = missing[0]
            cands = sorted((w for w in way_list if w.ind in seen
                            and w.instype == 'normal_way'),
                           key=lambda w: get_distance(wp1.loc, w.loc))
            linked = 0
            for wp2 in cands:
                path1, dist1 = self.prm.QueryTaskPlan(wp1.loc, wp2.loc)
                path2, dist2 = self.prm.QueryTaskPlan(wp2.loc, wp1.loc)
                if dist1 != -1 and wp2.ind not in wp1.connected:
                    wp1.distance.append(dist1)
                    wp1.connected.append(wp2.ind)
                    wp1.path.append(path1)
                if dist2 != -1 and wp1.ind not in wp2.connected:
                    wp2.distance.append(dist2)
                    wp2.connected.append(wp1.ind)
                    wp2.path.append(path2)
                if dist2 != -1:
                    linked += 1
                    if linked >= 2:
                        break
            if linked:
                print('env: reconnected unreachable waypoint %s via full '
                      'roadmap (%d links)' % (wp1.ind, linked))
            else:
                print('env: waypoint %s unreachable and could not be '
                      'reconnected' % wp1.ind)
                break

    def env_update(self, env_collision):
        """
        Env update with Collision Line
        env_collision = [special points, samples, collision type]
            special points = [[endpoint, entrypoints, ...], ... ] if narrow col
                             [[endpoint], [endpoint]]             if normal col
            samples = 1-D array
            collision type = 0 if narrow col, 1 if normal col
        """
        # print(env_collision)
        wpind = self.basic_way_num
        tind = 0

        # Initialize other things
        self.obj_ins['waypoint'] = self.obj_ins['waypoint'][:wpind]
        self.obj_ins['token'] = []
        for ele in self.obj_ins['waypoint']:
            ele.connected = []
            ele.distance = []
            ele.path = []
            ele.collision_connect = []
            ele.token_ind = None

        # PRM without collision samples
        samples = []
        roadmap = []
        residual_sample = []

        for collision in env_collision:
            col_entry = collision[0]
            col_samples = collision[1]
            col_type = collision[2]
            # region meta from the canonical merger (throat feeds the
            # size-aware capacity in the scheduling relaxation)
            col_meta = collision[3] if len(collision) > 3 else {}
            residual_sample += col_samples

            # Narrow Collision
            if col_type == 0:
                token = self.Instance('t'+str(tind), None, 'token')
                token.token_name = 'narrow_col_token'
                token.token_samples = col_samples
                token.throat = col_meta.get('throat')
                tind += 1

                temp_col_entry =[]
                for ele in col_entry:
                    temp_col_entry += ele

                # a basic waypoint inside a collision region becomes a region waypoint even when it is not an entry
                for i in range(self.basic_way_num):
                    way_loc = self.obj_ins['waypoint'][i].loc
                    if way_loc in col_samples and (not way_loc in temp_col_entry):
                        self.obj_ins['waypoint'][i].instype = 'col_way'
                        token.token_wp.append(self.obj_ins['waypoint'][i].ind)

                # entry waypoints as PDDL instances
                # one entry: a region waypoint; two entries: a region waypoint and a waiting waypoint
                for entry in col_entry:
                    if len(entry) == 1:
                        temp_wp1 = None
                        for i in range(self.basic_way_num):
                            if entry[0] == self.obj_ins['waypoint'][i].loc:
                                temp_wp1 = self.obj_ins['waypoint'][i]
                                temp_wp1.instype = 'col_way'
                                token.token_wp.append(temp_wp1.ind)
                                break
                        if not temp_wp1:
                            temp_wp1 = self.Instance('wp'+str(wpind), entry[0], 'col_way')
                            wpind +=1
                            token.token_wp.append(temp_wp1.ind)
                            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [temp_wp1]
                    elif len(entry) == 0:
                        # Now only think about entry points 1 or 2 for narrow col
                        raise Exception("Problem in Entry")
                    else:
                        temp_wp1 = None
                        for i in range(self.basic_way_num):
                            if entry[0] == self.obj_ins['waypoint'][i].loc:
                                temp_wp1 = self.obj_ins['waypoint'][i]
                                temp_wp1.instype = 'col_way'
                                token.token_wp.append(temp_wp1.ind)
                                break
                        if not temp_wp1:
                            temp_wp1 = self.Instance('wp'+str(wpind), entry[0], 'col_way')
                            wpind +=1
                            token.token_wp.append(temp_wp1.ind)
                            self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [temp_wp1]
                        for i in range(1, len(entry)):
                            temp_wp2 = None
                            for j in range(self.basic_way_num):
                                if entry[i] == self.obj_ins['waypoint'][j].loc:
                                    temp_wp2 = self.obj_ins['waypoint'][j]
                                    temp_wp2.instype = 'waiting_way'
                                    token.token_wp.append(temp_wp2.ind)
                                    break
                            if not temp_wp2:
                                temp_wp2 = self.Instance('wp'+str(wpind), entry[i], 'waiting_way')
                                wpind +=1
                                token.token_wp.append(temp_wp2.ind)
                                self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [temp_wp2]
                            temp_wp1.collision_connect.append(temp_wp2.ind)
                            temp_wp2.collision_connect.append(temp_wp1.ind)

                self.obj_ins['token'] = self.obj_ins.get('token', []) + [token]

            elif col_type == 1:
                temp_col_entry = []
                for ele in col_entry:
                    #samples += ele
                    temp_col_entry += ele

                token = self.Instance('t'+str(tind), None, 'token')
                token.token_name = 'normal_col_token'
                token.token_samples = col_samples
                token.throat = col_meta.get('throat')
                tind += 1

                for i in range(self.basic_way_num):
                    way_loc = self.obj_ins['waypoint'][i].loc
                    if way_loc in col_samples and (not way_loc in temp_col_entry):
                        self.obj_ins['waypoint'][i].instype = 'reserved_way'
                        self.obj_ins['waypoint'][i].token_ind = token.ind
                        token.token_wp.append(self.obj_ins['waypoint'][i].ind)

                for entry in col_entry:
                    temp_wp1 = None
                    for i in range(self.basic_way_num):
                        if entry[0] == self.obj_ins['waypoint'][i].loc:
                            temp_wp1 = self.obj_ins['waypoint'][i]
                            temp_wp1.instype = 'reserved_way'
                            temp_wp1.token_ind = token.ind
                            token.token_wp.append(temp_wp1.ind)
                            break
                    if not temp_wp1:
                        temp_wp1 = self.Instance('wp'+str(wpind), entry[0], 'reserved_way')
                        temp_wp1.token_ind = token.ind
                        wpind +=1
                        token.token_wp.append(temp_wp1.ind)
                        self.obj_ins['waypoint'] = self.obj_ins.get('waypoint', []) + [temp_wp1]
                self.obj_ins['token'] = self.obj_ins.get('token', []) + [token]

        # Same Token Connect
        for ele in self.obj_ins['token']:
            temp_token_way = ele.token_wp
            token_way = []
            if ele.token_name == 'narrow_col_token':
                for way in self.obj_ins['waypoint']:
                    if way.ind in temp_token_way and way.instype != 'waiting_way':
                        token_way.append(way)
            else:
                for way in self.obj_ins['waypoint']:
                    if way.ind in temp_token_way:
                        token_way.append(way)

            num_token_way = len(token_way)
            for i in range(num_token_way-1):
                for j in range(i+1, num_token_way):
                    way_loc1 = token_way[i].loc
                    way_loc2 = token_way[j].loc
                    temp_path, temp_dist = self.prm.QueryTaskPlan(way_loc1, way_loc2)
                    if temp_dist != -1:
                        buffered_line = LineString(temp_path).buffer(self.task_rr)
                        connect_flag = True
                        for another_way in token_way:
                        # for another_way in self.obj_ins['waypoint']:
                            if another_way.loc != way_loc1 and another_way.loc != way_loc2:
                                way_point = Point(another_way.loc).buffer(self.task_rr)
                                # way_point = Point(another_way.loc)
                                if buffered_line.intersects(way_point):
                                # if buffered_line.contains(way_point):
                                    connect_flag = False
                                    break
                        if connect_flag:
                            token_way[i].collision_connect.append(token_way[j].ind)
                            token_way[j].collision_connect.append(token_way[i].ind)


        # Making PRM without collision
        way_list = self.obj_ins['waypoint']
        way_len = len(way_list)
        for way in way_list:
            if way.instype != 'waiting_way':
                way_buffer = Point(way.loc).buffer(self.task_rr)
                for w in self.task_samples:
                    w_point = Point(w).buffer(self.task_rr)
                    if way_buffer.intersects(w_point):
                        residual_sample.append(w)
            else:
                samples.append(way.loc)

        residual_sample = set(residual_sample)
        for sample in self.task_samples:
            if not sample in residual_sample:
                samples.append(sample)

        skdtree = KDTree(samples)
        cd = self.prm.casting_dist[self.task_rr]
        nsample = len(samples)
        knn = min(int(len(self.task_samples)/40), 15)

        for (i, temp_point) in zip(range(nsample), samples):
            # bounded KNN (see env_instance / prm.generate_roadmap)
            for k in (min(nsample, 8 * knn + 1), nsample):
                index, dists = skdtree.search(temp_point, k=k)
                edge_id = []
                # 0 is self so start with 1
                for j in range(1, len(index)):
                    npoint = samples[index[j]]
                    if self.prm.isEdgeCollisionFree(temp_point, npoint, self.task_rr) and get_distance(temp_point, npoint) < cd:
                        edge_id.append(index[j])
                    if len(edge_id) >= knn:
                        break
                if len(edge_id) >= knn or k == nsample or dists[-1] >= cd:
                    break
            roadmap.append(edge_id)

        # hi = plot_prm_one(self.env, samples, roadmap, self.prm.goal_samples)
        # plt.show()

        for i in range(way_len-1):
            wp1 = way_list[i]
            for j in range(i+1, way_len):
                wp2 = way_list[j]
                if wp1.instype == 'col_way' and wp2.instype == 'col_way':
                    if wp2.ind in wp1.collision_connect:
                        path1, dist1 = self.prm.QueryTaskPlan(wp1.loc, wp2.loc)
                        path2, dist2 = self.prm.QueryTaskPlan(wp2.loc, wp1.loc)
                        if dist1 != -1:
                            wp1.distance.append(dist1)
                            wp1.connected.append(wp2.ind)
                            wp1.path.append(path1)
                        if dist2 != -1:
                            wp2.distance.append(dist2)
                            wp2.connected.append(wp1.ind)
                            wp2.path.append(path2)
                elif wp1.instype == 'col_way' and wp2.instype == 'waiting_way':
                    if wp2.ind in wp1.collision_connect:
                        path1, dist1 = self.prm.QueryTaskPlan(wp1.loc, wp2.loc)
                        path2, dist2 = self.prm.QueryTaskPlan(wp2.loc, wp1.loc)
                        if dist1 != -1:
                            wp1.distance.append(dist1)
                            wp1.connected.append(wp2.ind)
                            wp1.path.append(path1)
                        if dist2 != -1:
                            wp2.distance.append(dist2)
                            wp2.connected.append(wp1.ind)
                            wp2.path.append(path2)
                elif wp1.instype == 'waiting_way' and wp2.instype == 'col_way':
                    if wp1.ind in wp2.collision_connect:
                        path1, dist1 = self.prm.QueryTaskPlan(wp1.loc, wp2.loc)
                        path2, dist2 = self.prm.QueryTaskPlan(wp2.loc, wp1.loc)
                        if dist1 != -1:
                            wp1.distance.append(dist1)
                            wp1.connected.append(wp2.ind)
                            wp1.path.append(path1)
                        if dist2 != -1:
                            wp2.distance.append(dist2)
                            wp2.connected.append(wp1.ind)
                            wp2.path.append(path2)
                elif wp1.instype == 'reserved_way' and wp2.instype == 'reserved_way':
                    if wp2.ind in wp1.collision_connect:
                        path1, dist1 = self.prm.QueryTaskPlan(wp1.loc, wp2.loc)
                        path2, dist2 = self.prm.QueryTaskPlan(wp2.loc, wp1.loc)
                        if dist1 != -1:
                            wp1.distance.append(dist1)
                            wp1.connected.append(wp2.ind)
                            wp1.path.append(path1)
                        if dist2 != -1:
                            wp2.distance.append(dist2)
                            wp2.connected.append(wp1.ind)
                            wp2.path.append(path2)
                    elif wp1.token_ind != wp2.token_ind:
                        path1, dist1, path2, dist2 = self.prm.QuerywithRoadmap(wp1.loc, wp2.loc, samples, roadmap, skdtree, self.task_rr)
                        if dist1 != -1:
                            wp1.distance.append(dist1)
                            wp1.connected.append(wp2.ind)
                            wp1.path.append(path1)
                        if dist2 != -1:
                            wp2.distance.append(dist2)
                            wp2.connected.append(wp1.ind)
                            wp2.path.append(path2)
                elif wp1.instype == 'waiting_way' and wp2.instype == 'waiting_way':
                    path1, dist1, path2, dist2 = self.prm.QuerywithRoadmap(wp1.loc, wp2.loc, samples, roadmap, skdtree, self.task_rr)
                    if dist1 != -1:
                        wp1.distance.append(dist1)
                        wp1.connected.append(wp2.ind)
                        wp1.path.append(path1)
                    if dist2 != -1:
                        wp2.distance.append(dist2)
                        wp2.connected.append(wp1.ind)
                        wp2.path.append(path2)
                elif wp1.instype == 'normal_way' and wp2.instype == 'col_way':
                    pass
                elif wp1.instype == 'col_way' and wp2.instype == 'normal_way':
                    pass
                elif wp1.instype == 'reserved_way' and wp2.instype == 'col_way':
                    pass
                elif wp1.instype == 'col_way' and wp2.instype == 'reserved_way':
                    pass
                else:
                    path1, dist1, path2, dist2 = self.prm.QuerywithRoadmap(wp1.loc, wp2.loc, samples, roadmap, skdtree, self.task_rr)
                    if dist1 != -1:
                        wp1.distance.append(dist1)
                        wp1.connected.append(wp2.ind)
                        wp1.path.append(path1)
                    if dist2 != -1:
                        wp2.distance.append(dist2)
                        wp2.connected.append(wp1.ind)
                        wp2.path.append(path2)

        # Completeness guard: dense collision regions can fragment the
        # reduced roadmap until goal waypoints are cut off from the robot
        # starts (see _reconnect_unreachable).
        self._reconnect_unreachable()

        for ele in self.obj_ins['waypoint']:
            if ele.instype == 'waiting_way':
                ele.instype = 'col_way'
            if ele.instype == 'reserved_way':
                ele.instype = 'col_way'

        for ele in self.obj_ins['waypoint']:
            if ele.instype == 'col_way':
                ele.reserved = True

    def generate_header(self):
        """
        Make Header for Problem PDDL File
        Header includes domain definition and objects
        """
        # Domain Definition
        self.pFile.write('(define (problem ' + getattr(self, 'problemName', self.pFilename.replace('.pddl', '')) + ')\n')
        self.pFile.write('(:domain ' + self.domain + ')\n')
        self.pFile.write('(:objects\n')

        # Object Adding
        # They are ordered by instype not ins (for example weld_robot, bolt_robot not robot)
        for name, obj in self.obj_ins.items():
            temp_dict = dict()
            for ele in obj:
                temp_dict[ele.instype] = temp_dict.get(ele.instype, []) + [ele.ind]
            for obj_type, ind in temp_dict.items():
                if obj_type == 'token':
                    break
                self.pFile.write('        ')
                counter = 1
                for ele in ind:
                    self.pFile.write(ele + ' ')
                    if counter == len(ind):
                        self.pFile.write('- ' + obj_type + '\n')
                    elif counter % 5 == 0:
                        self.pFile.write('- ' + obj_type + '\n')
                        self.pFile.write('        ')
                    counter += 1
        self.pFile.write(')\n')

    def parse_pddl_plan(self, planFile):
        """
        Plan Reading and Parse Function. They only parse the plan, no refine
        planfile : Plan from optic wrapper
        @Return pair of plan dispatch (robot-goals)
        @Retrun Path Data Structure like
        [point-Tuple(float, float), dispatch time -float, action_time - float]
        """
        f = Path(planFile)
        if not f.is_file():
            raise BaseException(planFile + " does not exists.")
        f = open(planFile, 'r')
        lines = f.readlines()
        path = dict()
        #robot_dict = dict()
        way_dict = dict()
        cost = 0
        cal_time = 0
        for r in self.obj_ins['robot']:
            path[r.ind] = []
        for w in self.obj_ins['waypoint']:
            way_dict[w.ind] = dict()
            way_dict[w.ind]['loc'] = w.loc
            for i in range(len(w.connected)):
                way_dict[w.ind][w.connected[i]] = w.path[i]

        for line in lines:
            line = line.strip()
            if 'metric' in line:
                ind = line.find('metric')
                line = line.replace('metric ', '')
                cost = float(line[ind:])
                #print('cost: ', cost)
            if 'Time' in line:
                ind = line.find('Time')
                line = line.replace('Time ','')
                cal_time = float(line[ind:])
                #print('cal time: ', cal_time)
            if '_navigate' in line:
                # match robot names on word boundaries (robot1 must not match robot10)
                for r in path:
                    if ' ' + r + ' ' in line:
                        # Dispatch Time
                        time_ind = line.find(':')
                        dispatch_t = float(line[:time_ind])
                        # Waypoint 1
                        wp1_ind = line.find('wp')
                        sub1_line = line[wp1_ind:]
                        space_ind = sub1_line.find(' ')
                        wp1 = sub1_line[:space_ind]
                        # Waypoint 2
                        sub2_line = line[wp1_ind+2:]
                        wp2_ind = sub2_line.find('wp')
                        sub2_line = sub2_line[wp2_ind:]
                        blanket_ind = sub2_line.find(')')
                        space_ind = sub2_line.find(' ')
                        wp2 = sub2_line[:min(space_ind, blanket_ind)]
                        # Path
                        local_path = None
                        if wp1 in way_dict:
                            if wp2 in way_dict[wp1]:
                                local_path = copy.deepcopy(way_dict[wp1][wp2])
                        if not local_path:
                            raise Exception("something wrong in local_path: "
                                            "file=%s robot=%s wp1=%r wp2=%r line=%r"
                                            % (planFile, r, wp1, wp2, line))
                        # Action duration
                        bf_ind = line.find('[')
                        be_ind = line.find(']')
                        action_d = float(line[bf_ind+1:be_ind])
                        path[r].append([local_path, dispatch_t, action_d])

            if 'do_task_single' in line:
                for r in path:
                    if ' ' + r + ' ' in line:
                        # Dispatch Time
                        time_ind = line.find(':')
                        dispatch_t = float(line[:time_ind])
                        # Waypoint 1
                        wp1_ind = line.find('wp')
                        sub1_line = line[wp1_ind:]
                        space_ind = sub1_line.find(' ')
                        wp1 = sub1_line[:space_ind]
                        if wp1 in way_dict:
                            wp1 = way_dict[wp1]['loc']
                        goal_ind = line.find('goal')
                        b_ind = line.find(')')
                        line2 = line[goal_ind:b_ind]
                        s_ind = line2.find(' ')
                        goal_name = line2[:s_ind]
                        task_name = line2[s_ind+1:]

                        bf_ind = line.find('[')
                        be_ind = line.find(']')
                        action_d = float(line[bf_ind+1:be_ind])
                        path[r].append([[wp1], dispatch_t, action_d, task_name, goal_name])
            if 'do_task_double' in line:
                for r in path:
                    if ' ' + r + ' ' in line:
                        # Dispatch Time
                        time_ind = line.find(':')
                        dispatch_t = float(line[:time_ind])
                        # Waypoint 1
                        robot_ind = line.find(r)
                        robot_line = line[robot_ind:]
                        wp_ind = robot_line.find('wp')
                        wp_line = robot_line[wp_ind:]
                        space_ind = wp_line.find(' ')
                        wp1 = wp_line[:space_ind]
                        if wp1 in way_dict:
                            wp1 = way_dict[wp1]['loc']

                        # for goal
                        goal_ind = line.find('goal')
                        b_ind = line.find(')')
                        line2 = line[goal_ind:b_ind]
                        s_ind = line2.find(' ')
                        goal_name = line2[:s_ind]
                        task_name = line2[s_ind+1:]

                        bf_ind = line.find('[')
                        be_ind = line.find(']')
                        action_d = float(line[bf_ind+1:be_ind])
                        path[r].append([[wp1], dispatch_t, action_d, task_name, goal_name])
        f.close()

        return path, cost, cal_time

    def generate_path(self, parsed_path):
        """
        parsed_path to robot_path
        @Retrun Path Data Structure like
        """
        robot_path = dict()
        total_cost = 0
        max_cost = 0
        for name, ele in parsed_path.items():
            combined_path = dict()
            path_cost = []
            path_only = []
            start_waiting = 0
            tstart_waiting = []
            plan_num = len(ele)
            stop_time = 0
            for i in range(plan_num):
                if i == plan_num-1:     # Last Action
                    curr_plan = ele[i][0]
                    curr_time = ele[i][1]
                    curr_action = ele[i][2]
                    action_cost = 0
                    taction_cost = []

                    if not path_only and curr_time > 1e-6:
                        start_waiting += curr_time
                        tstart_waiting += [(curr_time, 2)]

                    if path_only and len(curr_plan) == 1:
                        if path_only[-1] != curr_plan[0]:
                            raise Exception("Service Location Error!")
                        else:
                            action_cost += curr_action
                            taction_cost += [(curr_action, ele[i][3], ele[i][4])]
                            if type(path_cost[-1]) == list:
                                path_cost[-1][1] += action_cost
                                path_cost[-1][2] += taction_cost
                                stop_time += action_cost
                            else:
                                path_cost[-1] = [path_cost[-1], action_cost, taction_cost]
                                stop_time += action_cost
                    elif not path_only and len(curr_plan) == 1:
                        start_waiting += (curr_action + action_cost)
                        tstart_waiting += [(curr_action, ele[i][3], ele[i][4])]
                        path_only = curr_plan
                        path_cost = [curr_plan[0], start_waiting, tstart_waiting]
                        stop_time += start_waiting
                    else:
                        temp_path = copy.deepcopy(curr_plan)
                        if path_only:
                            path_only += curr_plan[1:]
                            path_cost += temp_path[1:]
                            if action_cost >0:
                                path_cost[-1] = [path_cost[-1], action_cost, taction_cost]
                                stop_time += action_cost
                        else:
                            path_only = curr_plan
                            path_cost = temp_path
                            if action_cost > 0:
                                path_cost[-1] = [path_cost[-1], action_cost, taction_cost]
                                stop_time += action_cost
                            if start_waiting > 0:
                                path_cost[0] = [path_cost[0], start_waiting, tstart_waiting]
                                stop_time += start_waiting
                else:             # Not Last Action
                    curr_plan = ele[i][0]
                    curr_time = ele[i][1]
                    curr_action = ele[i][2]
                    if not curr_plan:
                        print(parsed_path)
                        raise Exception("Something Wrong in Parsed path")

                    next_time = ele[i+1][1]

                    action_cost = 0
                    taction_cost = []
                    # Waiting Time means idle time for robot
                    waiting_time = next_time - (curr_time + curr_action)

                    if waiting_time > 1e-6:
                        action_cost += waiting_time
                        taction_cost += [(waiting_time, 2)]

                    if not path_only and curr_time > 1e-6:   # Start Action with some waiting
                        start_waiting += curr_time
                        tstart_waiting += [(curr_time, 2)]

                    # If weld or bolt
                    if path_only and len(curr_plan) == 1:
                        if path_only[-1] != curr_plan[0]:
                            raise Exception("Service Location Error!")
                        else:
                            action_cost += curr_action
                            taction_cost = [(curr_action, ele[i][3], ele[i][4])] + taction_cost
                            if type(path_cost[-1]) == list:
                                path_cost[-1][1] += action_cost
                                path_cost[-1][2] += taction_cost
                                stop_time += action_cost
                            else:
                                path_cost[-1] = [path_cost[-1], action_cost, taction_cost]
                                stop_time += action_cost
                    # Robot starts with weld or bolt action
                    elif not path_only and len(curr_plan) == 1:
                        start_waiting += (curr_action + action_cost)
                        tstart_waiting += [(curr_action, ele[i][3], ele[i][4])]
                    # Navigation action
                    else:
                        temp_path = copy.deepcopy(curr_plan)
                        if path_only:
                            path_only += curr_plan[1:]
                            path_cost += temp_path[1:]
                            if action_cost > 0:
                                path_cost[-1] = [path_cost[-1], action_cost, taction_cost]
                                stop_time += action_cost
                        else:
                            path_only = curr_plan
                            path_cost = temp_path
                            if action_cost > 0:
                                path_cost[-1] = [path_cost[-1], action_cost, taction_cost]
                                stop_time += action_cost
                            if start_waiting > 0:
                                path_cost[0] = [path_cost[0], start_waiting, tstart_waiting]
                                stop_time += start_waiting
            if path_only:
                combined_path['path_cost'] = path_cost
                combined_path['path_only'] = path_only
                combined_path['path_time'] = stop_time + get_sum_of_cost(path_only)
                max_cost = max(max_cost, combined_path['path_time'])
                total_cost += (get_sum_of_cost(path_only) + stop_time)
                robot_path[name] = combined_path

        for name, ele in self.robots.items():
            if name in robot_path:
                if robot_path[name]['path_time'] != max_cost:
                    time_remain = max_cost - robot_path[name]['path_time']
                    if type(robot_path[name]['path_cost'][-1]) == list:
                        robot_path[name]['path_cost'][-1][1] += time_remain
                        robot_path[name]['path_time'] = max_cost
                        robot_path[name]['path_cost'][-1][2] += [(time_remain, 2)]
                    else:
                        robot_path[name]['path_cost'][-1] = [robot_path[name]['path_cost'][-1], time_remain, [(time_remain, 2)]]
                        robot_path[name]['path_time'] = max_cost
            else:
                path_only = [self.robots[name].pos]
                path_cost = [[self.robots[name].pos, max_cost, [(max_cost, 2)]]]
                robot_path[name] = dict()
                robot_path[name]['path_only'] = path_only
                robot_path[name]['path_cost'] = path_cost
                robot_path[name]['path_time'] = max_cost

        return robot_path, total_cost, max_cost



def read_domain(dFile):
    """
    Domain Reading Func
    dFile: Domain File Path
    @Return domain_name(string), domain type(int)
    """
    f = Path(dFile)
    if not f.is_file():
        raise BaseException(dFile + " does not exists.")
    f = open(dFile, 'r')
    lines = f.readlines()
    domain_name = ''

    for line in lines:
        line = line.strip()
        # if we need more info from domain file, remove break and read more lines here.
        if line.startswith('(define'):
            start = line.find('domain ')
            line = line.replace('domain ','')
            end = line.find(')')
            domain_name = line[start:end]
            break
    f.close()

    return domain_name






