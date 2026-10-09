import heapq
import math
import random
from common.utils import *
from shapely.geometry import Point, LineString


class PRMPlanning():
    def __init__(self, environment):
        """
 Ininital Probabilistic RoadMap
 Env: Enviroment instance from yaml file
 Bounds: Boundary of the map
 Robot: Robot configurations
 Samples: Dict (robot_radius - 1-D array samples)
 Roadmap: Dict (robot_radius - 2-D array Roadmap)
 skdtree: KDTree for finding nearest samples (Dict)
 """
        self.env = environment
        self.bounds = environment.bounds
        self.robots = environment.robots
        self.robots_map = environment.robots_map
        self.num_of_robot = environment.num_of_robot
        # Vestigial (always empty since Voronoi removal): collision_vor's
        # endpoint handling still iterates it.
        self.vor_ways = []
        # Per-radius clearance of each sample (distance to nearest obstacle
        # or boundary): the geometric raw material for narrow-passage
        # classification, region throat width and size-aware capacities.
        self.clearances = {}
        # --smooth: deterministic shortcutting of full-roadmap query paths
        self.smooth = False
        self.samples = {}
        self.skdtree = {}
        self.roadmaps = {}
        self.sampling_dist = {}
        self.casting_dist = {}

        self.init_num_of_samples = environment.init_num_of_samples
        if self.init_num_of_samples == 0:
            # no sample budget in the scenario: about one sample per free square metre
            self.init_num_of_samples = max(200, min(int(environment.free_area), 8000))

        self.max_rr = 0
        for name, robot in self.robots_map.items():
            if self.max_rr < robot.robot_radius:
                self.max_rr = robot.robot_radius
        self.task_samples = []
        self.task_roadmap = []
        # Obstacles never change within a PRM instance, so edge collision
        # results are memoized (env_update rebuilds roadmaps every iteration
        # and re-tests the same sample pairs).
        self._edge_free_cache = {}
        self._obs_prep_cache = {}
        # Be careful about goal_samples are 2D array
        self.goal_samples = []
        # self.max_skdtree
        self.knn_const = 40

    def dijkstra_planning(self, start, goal, road_map, sample_points):
        """
 start: start position (x,y)
 goal: goal position (x,y)
 road_map: roadmap by generate_roadmap (2d array)
 sample_points: sampled location by sample_waypoint (array)
 @return: lists of path coordinates ([(x1,y1), (x2,y2) ...]), empty list when no path was found
 """
        start_index = sample_points.index(start)
        root = {'id': start_index, 'loc': start, 'cost': 0.0, 'parent': None}
        best = {start_index: root}
        seq = 0
        heap = [(0.0, seq, root)]
        while heap:
            cost, _, curr = heapq.heappop(heap)
            if cost > best[curr['id']]['cost']:
                continue  # stale entry
            if curr['loc'] == goal:
                return get_path(curr)
            for child_id in road_map[curr['id']]:
                child_loc = sample_points[child_id]
                d = math.hypot(child_loc[0] - curr['loc'][0], child_loc[1] - curr['loc'][1])
                new_cost = cost + d
                known = best.get(child_id)
                if known is None or new_cost < known['cost']:
                    child = {'id': child_id, 'loc': child_loc,
                             'cost': new_cost, 'parent': curr}
                    best[child_id] = child
                    seq += 1
                    heapq.heappush(heap, (new_cost, seq, child))
        return None # Failed to find solution

    def ConstructPhase(self, goals_map):
        """
 Construct Roadmap Function
 Goal Related Sampling - Region Based Goal and Obstacle Based Goal

 Sampling - Making KDTree with samples - Constructing Roadmap
 Every different Model uses different Roadmap (Check it with there is key in dict)
 Sampling Distance: Minimum distance between samples
 Casting Distance: Maximum distance connecting the samples

 @Return initial samples and roadmap
 """

        # Setting number of samples
        # Honor the scenario's sample budget on large maps; the old 2000
        # ceiling silently clamped e.g. a 4000-sample warehouse request.
        sample_n = min(max(len(self.env.obstacles)*2, self.init_num_of_samples), 8000)
        self.init_sample_n = sample_n
        goal_samples = []

        # Goal Samples
        for name, description in goals_map.items():
            if description['based'] == 'region':
                goals_map[name]['samples'] = self.region_goal_samples(description['shape'])
                goal_samples.append(goals_map[name]['samples'])
            elif description['based'] == 'obstacle':
                goals_map[name]['samples'] = self.obs_goal_samples(description['shape'])
                goal_samples.append(goals_map[name]['samples'])
            else:
                raise Exception("Goal must be explicted by based")
        self.goal_samples = goal_samples
        # PRM structure
        for name, robot in self.robots_map.items():
            rr = robot.robot_radius
            if not rr in self.roadmaps:
                if self.env.debug_mode:
                    print("Construct Roadmap for robot radius %0.2f"%rr)
                # For Sampling Generalization
                sd = self.env.free_area / (rr*rr*math.pi)
                sd = round(sd /sample_n,2)
                if sd < 0.5:
                    sd = 0.5
                elif sd > 2.0:
                    sd = 2.0
                self.sampling_dist[rr] = sd * rr
                                # connection radius from the robot scale (6x the sampling distance)
                cd = 6 * self.sampling_dist[rr]
                if cd < 0.75:
                    cd = 0.75
                elif cd > 2.25:
                    cd = 2.25
                self.casting_dist[rr] = cd
                self.samples[rr] = self.sample_waypoint(rr, sample_n, self.sampling_dist[rr])
                self.clearances[rr] = self.env.clearance_many(self.samples[rr])
                self.skdtree[rr] = KDTree(self.samples[rr])
                self.roadmaps[rr] = self.generate_roadmap(rr, self.casting_dist[rr])
            if not self.isRoadmapEnough(robot.pos, goal_samples, rr):
                raise Exception("Goal cannot be reached!")
        # Setting PRM only for Task Domain
        self.build_task_roadmap(goal_samples)

        return self.samples, self.roadmaps, goal_samples

    def attach_point(self, point, samples, skdtree, rr):
        """Connect a free point into a roadmap the way add_single_query does:
 up to knn collision-free neighbors within casting distance.
 @Return list of neighbor sample indices"""
        knn = min(len(self.samples[rr])/self.knn_const, 15)
        cd = self.casting_dist[rr]
        index, dists = skdtree.search(point, k=knn)
        edge_id = []
        for i in range(0, len(index)):
            npoint = samples[index[i]]
            if self.isEdgeCollisionFree(point, npoint, rr) and get_distance(point, npoint) < cd:
                edge_id.append(index[i])
            if len(edge_id) >= knn:
                break
        return edge_id

    def dijkstra_all(self, start, start_edges, road_map, sample_points):
        """Single-source Dijkstra from a virtual start attached to the roadmap
 through start_edges (from attach_point). Runs to exhaustion; returns
 {sample_index: node} whose paths are read with get_path(node).
 Because the start is virtual and destinations are closed off through
 their own attachment edges, per-pair results match dijkstra_planning
 on a roadmap where only that pair was inserted."""
        root = {'id': -1, 'loc': start, 'cost': 0.0, 'parent': None}
        best = {}
        heap = []
        seq = 0
        for child_id in start_edges:
            child_loc = sample_points[child_id]
            d = math.hypot(child_loc[0] - start[0], child_loc[1] - start[1])
            known = best.get(child_id)
            if known is None or d < known['cost']:
                child = {'id': child_id, 'loc': child_loc, 'cost': d, 'parent': root}
                best[child_id] = child
                seq += 1
                heapq.heappush(heap, (d, seq, child))
        while heap:
            cost, _, curr = heapq.heappop(heap)
            if cost > best[curr['id']]['cost']:
                continue
            for child_id in road_map[curr['id']]:
                child_loc = sample_points[child_id]
                d = math.hypot(child_loc[0] - curr['loc'][0], child_loc[1] - curr['loc'][1])
                new_cost = cost + d
                known = best.get(child_id)
                if known is None or new_cost < known['cost']:
                    child = {'id': child_id, 'loc': child_loc, 'cost': new_cost, 'parent': curr}
                    best[child_id] = child
                    seq += 1
                    heapq.heappush(heap, (new_cost, seq, child))
        return best

    def QuerywithRoadmap(self, start_pos, goal_pos, samples, roadmap, skdtree, rr):
        if not start_pos in samples or not goal_pos in samples:
            query_samples, query_roadmap = self.add_single_query(start_pos, goal_pos, samples, roadmap, rr, skdtree=skdtree)
            if start_pos == goal_pos:
                path1 = [start_pos, goal_pos]
                path2 = [goal_pos, start_pos]
            elif get_distance(start_pos, goal_pos) < rr * 5 and self.isEdgeCollisionFree(start_pos, goal_pos, rr):
                task_path1, task_cost1 = self.QueryTaskPlan(start_pos,goal_pos)
                task_path2, task_cost2 = self.QueryTaskPlan(goal_pos,start_pos)
                if task_cost1 != -1:
                    path1 = task_path1
                else:
                    path1 = [start_pos, goal_pos]
                if task_cost2 != -1:
                    path2 = task_path2
                else:
                    path2 = [goal_pos, start_pos]
            else:
                path1 = self.dijkstra_planning(start_pos, goal_pos, query_roadmap, query_samples)
                # if path1:
                # path1 = self.path_smoothing(path1, rr)
                path2 = self.dijkstra_planning(goal_pos, start_pos, query_roadmap, query_samples)
                # if path2:
                # path2 = self.path_smoothing(path2, rr)
        else:
            if start_pos == goal_pos:
                path1 = [start_pos, goal_pos]
                path2 = [goal_pos, start_pos]
            elif get_distance(start_pos, goal_pos) < rr * 5 and self.isEdgeCollisionFree(start_pos, goal_pos, rr):
                task_path1, task_cost1 = self.QueryTaskPlan(start_pos,goal_pos)
                task_path2, task_cost2 = self.QueryTaskPlan(goal_pos,start_pos)
                if task_cost1 != -1:
                    path1 = task_path1
                else:
                    path1 = [start_pos, goal_pos]
                if task_cost2 != -1:
                    path2 = task_path2
                else:
                    path2 = [goal_pos, start_pos]
            else:
                path1 = self.dijkstra_planning(start_pos, goal_pos, roadmap, samples)
                # if path1:
                # path1 = self.path_smoothing(path1, rr)
                path2 = self.dijkstra_planning(goal_pos, start_pos, roadmap, samples)
                # if path2:
                # path2 = self.path_smoothing(path2, rr)

        if path1 is None or path2 is None:
            cost1 = -1
            cost2 = -1
        else:
            cost1 = round(get_sum_of_cost(path1), 2)
            cost2 = round(get_sum_of_cost(path2), 2)
            if cost1 < cost2:
                path2 = list(path1)
                path2.reverse()
                cost2 = round(get_sum_of_cost(path2), 2)
            else:
                path1 = list(path2)
                path1.reverse()
                cost1 = round(get_sum_of_cost(path1), 2)

        return path1, cost1, path2, cost2

    def prune(self, path, rr):
        """Deterministic greedy shortcutting: from each kept vertex jump to
 the furthest later vertex reachable by a collision-free straight
 segment. Checks static obstacles only, so apply it to full-roadmap
 paths; reduced-roadmap paths (which detour collision regions on
 purpose) must keep their via structure.

 The pruned length is what callers store as (distance w1 w2), so the
 PDDL durations, relaxation travel times, and executed trajectories
 all see the same geometry."""
        if not self.smooth or not path or len(path) <= 2:
            return path
        out = [path[0]]
        i, n = 0, len(path)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self.isEdgeCollisionFree(path[i], path[j], rr):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def QueryTaskPlan(self, start_pos, goal_pos):
        #print(len(self.task_samples), len(self.task_roadmap))
        path = None
        if start_pos == goal_pos:
            path = [start_pos, goal_pos]
        elif self.isEdgeCollisionFree(start_pos, goal_pos, self.max_rr) and \
            get_distance(start_pos, goal_pos) < self.max_rr*3:
            #print("No Edge Collision occurs")
            path = [start_pos, goal_pos]
        else:
            samples = self.task_samples
            roadmap = self.task_roadmap
            if start_pos not in samples or goal_pos not in samples:
                # Collision-derived waypoints (entry/waiting points) are new
                # configurations that were never inserted into the task
                # roadmap; attach them on a copy before searching.
                samples, roadmap = self.add_single_query(start_pos, goal_pos,
                                                         samples, roadmap, self.max_rr)
            path = self.dijkstra_planning(start_pos, goal_pos, roadmap, samples)
            path = self.prune(path, self.max_rr)
        if path:
            cost = get_sum_of_cost(path)
            return path, round(cost,2)
        else:
            return None, -1

    def isRoadmapEnough(self, start_pos, goal_samples, rr):
        """
 Check reach every goal sample from start position
 start_pos = one robot start position
 goal_samples = every goal_samples
 rr = Robot Radius
 @Return True if every goal samples are reached from position, else reconstruction or False
 """
        start = [start_pos]
        goals = []
        for ele in goal_samples:
            goals += ele

        queried_samples, queried_roadmap = self.add_query(start, goals, rr)
        for goal in goals:
            path = None
            if start_pos == goal:
                path = [start_pos, goal]
            while path is None:
                # Growth budget must scale with the initial sample count:
                # the sample cap grows with the map so that large maps get at least one reconstruction
                cap = max(1500, 2 * getattr(self, 'init_sample_n', 750))
                if len(self.samples[rr]) > cap:
                    print("Already too many samples")
                    return False
                path = self.dijkstra_planning(start_pos, goal, queried_roadmap, queried_samples)
                if path is None:
                    print("Roadmap Reconstruction")
                    sd = self.sampling_dist[rr]
                    cd = self.casting_dist[rr]
                    self.samples[rr], self.roadmaps[rr] = self.roadmap_reconstruct(self.samples[rr], rr, sd, cd)
                    queried_samples, queried_roadmap = self.add_query(start, goals, rr)
        return True

    def roadmap_reconstruct(self, samples, rr, sd, cd):
        """
 Roadmap Reconstruct Function
 samples: Existed Sample
 rr: Robot Radius
 sd: Sampling Distance
 cd: Casting Distance
 @Return 1-D array 100 num added sample, 2-D array corresponding Roadmap
 """
        sample_points = list(samples)
        # resampling growth proportional to the map size
        nsample = len(samples) + max(100, len(samples) // 8)
        # half bridge / half uniform resampling; grid-hash rejection of close samples
        grid = {}
        for p in sample_points:
            grid.setdefault((int(p[0] // sd), int(p[1] // sd)), []).append(p)

        def far_enough(q):
            gx, gy = int(q[0] // sd), int(q[1] // sd)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for p in grid.get((gx + dx, gy + dy), ()):
                        if get_distance(p, q) < sd:
                            return False
            return True

        target_bridge = len(sample_points) + (nsample - len(sample_points)) // 2
        while len(sample_points) < nsample:
            if len(sample_points) < target_bridge:
                q = self.find_bridge_collision_free_configuration(rr)
            else:
                q = self.find_random_collision_free_configuration(rr)
            if far_enough(q):
                sample_points.append(q)
                grid.setdefault((int(q[0] // sd), int(q[1] // sd)), []).append(q)
        # KDTree for NN
        knn = min(int(nsample / self.knn_const), 15)
        re_roadmap = []
        self.skdtree[rr] = KDTree(sample_points)
        skdtree = self.skdtree[rr]
        # Making Roadmap (bounded KNN, same widening rule as generate_roadmap)
        for (i, temp_point) in zip(range(nsample), sample_points):
            for k in (min(nsample, 8 * knn + 1), nsample):
                index, dists = skdtree.search(temp_point, k=k)
                edge_id = []
                # 0 is self so start with 1
                for j in range(1, len(index)):
                    npoint = sample_points[index[j]]
                    if self.isEdgeCollisionFree(temp_point, npoint, rr) and get_distance(temp_point, npoint) < cd:
                        edge_id.append(index[j])
                    if len(edge_id) >= knn:
                        break
                if len(edge_id) >= knn or k == nsample or dists[-1] >= cd:
                    break
            re_roadmap.append(edge_id)

        return sample_points, re_roadmap

    def add_single_query(self, start, goal, samples, roadmap, rr, skdtree = None):
        # rows hold ints/tuples only - a per-row shallow copy is enough
        # (the old deepcopy dominated query time on dense roadmaps)
        query_samples = list(samples)
        query_roadmap = [list(row) for row in roadmap]
        knn = min(len(self.samples[rr])/self.knn_const, 15)
        sample_len = len(samples)
        cd = self.casting_dist[rr]

        if skdtree is None:
            skdtree = self.skdtree[rr]

        if not start in query_samples:
            query_samples.append(start)
            index, dists = skdtree.search(start, k = knn)
            edge_id = []
            for i in range(0, len(index)):
                npoint = samples[index[i]]
                if self.isEdgeCollisionFree(start, npoint, rr) and get_distance(start, npoint) < cd:
                    edge_id.append(index[i])
                if len(edge_id) >= knn:
                    break
            query_roadmap.append(edge_id)

            for ele in edge_id:
                query_roadmap[ele].append(len(query_samples)-1)

        if not goal in query_samples:
            query_samples.append(goal)
            index, dists = skdtree.search(goal, k = knn)
            edge_id = []
            for i in range(0, len(index)):
                npoint = samples[index[i]]
                if self.isEdgeCollisionFree(goal, npoint, rr) and get_distance(goal, npoint) < cd:
                    edge_id.append(index[i])
                if len(edge_id) >= knn:
                    break
            query_roadmap.append(edge_id)

            for ele in edge_id:
                query_roadmap[ele].append(len(query_samples)-1)

        return query_samples, query_roadmap

    def add_query(self, starts, goals, rr):
        """
 Adding start and goal position to given sample and roadmap
 Add Query means they are using existed sample and roadmap.
 start: list of start positions of robots
 goal: list of goal positions
 samples: 1-D array Samples
 rr: Robot Radius
 @Return 1-D array queried samples, 2-D array queried roadmap
 """
        samples = self.samples[rr]
        roadmap = self.roadmaps[rr]
        query_samples = list(samples)
        query_roadmap = [list(row) for row in roadmap]
        skdtree = self.skdtree[rr]
        cd = self.casting_dist[rr]

        # Add start pos and goal pos to sample
        add_sample = []
        temp_samples = starts+goals
        for point in temp_samples:
            if not point in query_samples:
                add_sample.append(point)
        query_samples += add_sample
        nsample = len(query_samples)

        knn = min(int(nsample/self.knn_const), 15)

        sample_len = len(samples)
        for sample in add_sample:
            index, dists = skdtree.search(sample, k = knn)
            edge_id = []
            for i in range(0, len(index)):
                npoint = samples[index[i]]
                if self.isEdgeCollisionFree(sample, npoint, rr) and get_distance(sample, npoint) < cd:
                    edge_id.append(index[i])
                if len(edge_id) >= knn:
                    break
            query_roadmap.append(edge_id)
            for ele in edge_id:
                query_roadmap[ele].append(sample_len)
            sample_len += 1

        return query_samples, query_roadmap

    def sample_waypoint(self, rr, sample_n, sd):
        """
 Sampling Waypoint Function
 rr: Robot radius
 sample_n: Number of Samples
 sd: Minimum Sampling Distance between samples
 @Return 1-D array Samples
 """

        # 1/3 bridge sampling (narrow passages) + 2/3 uniform, blue-noise
        # spaced by sd. The min-distance rejection uses a grid hash: the
        # old full linear scan per candidate was O(n^2) overall.
        sample_points = []
        grid = {}

        def far_enough(q):
            gx, gy = int(q[0] // sd), int(q[1] // sd)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for p in grid.get((gx + dx, gy + dy), ()):
                        if get_distance(p, q) < sd:
                            return False
            return True

        def accept(q):
            sample_points.append(q)
            grid.setdefault((int(q[0] // sd), int(q[1] // sd)), []).append(q)

        while len(sample_points) < sample_n / 3:
            q = self.find_bridge_collision_free_configuration(rr)
            if far_enough(q):
                accept(q)
        while len(sample_points) < sample_n:
            q = self.find_random_collision_free_configuration(rr)
            if far_enough(q):
                accept(q)

        return sample_points

    def region_goal_samples(self, polygon):
        """
 Goal Region Sampling
 polygon input
 @Return goal samples inside region
 """
        sample_points = []
        sample_n = 1+(polygon.area//((self.max_rr*2.0)**2))
        sd = polygon.area / sample_n
        sample_points.append(polygon.centroid.coords[0])
        minx = min(polygon.exterior.coords.xy[0])
        maxx = max(polygon.exterior.coords.xy[0])
        miny = min(polygon.exterior.coords.xy[1])
        maxy = max(polygon.exterior.coords.xy[1])
        bounds = (minx, miny, maxx, maxy)
        #return sample_points

        count = 0
        while len(sample_points) < sample_n:
            if count > 200 and len(sample_points) > 0:
                break
            elif count > 500:
                raise Exception("Can't sample in Goal Region")
            casting_flag = False
            q = self.get_random_point(bounds)
            for sample in sample_points:
                if get_distance(sample, q) < sd*3.0: # It's important but depends on too much paramater
                    casting_flag = True
                    break
            if not self.isPointCollisionFree(q, self.max_rr) or self.isOutOfBounds(q, self.max_rr):
                casting_flag = True
            if casting_flag == False:
                sample_points.append(q)
            count += 1
        return sample_points

    def obs_goal_samples(self, polygon):
        """
 Goal Obstacle Sampling
 polygon input
 @Return goal samples near goal obstacle
 """
        sample_points = []
        sample_n = 1+ (polygon.area//((self.max_rr*2)**2))
        access_dist = self.max_rr*2.0
        minx = min(polygon.exterior.coords.xy[0]) - access_dist
        maxx = max(polygon.exterior.coords.xy[0]) + access_dist
        miny = min(polygon.exterior.coords.xy[1]) - access_dist
        maxy = max(polygon.exterior.coords.xy[1]) + access_dist
        bounds = (minx, miny, maxx, maxy)

        count = 0
        while len(sample_points) < sample_n:
            if count > 200 and len(sample_points) > 0:
                break
            elif count > 500:
                raise Exception("Can't sample in Goal Obstalces")
            q = self.get_random_point(bounds)
            if self.isPointCollisionFree(q, self.max_rr) and not self.isOutOfBounds(q, self.max_rr):
                casting_flag = False
                for sample in sample_points:
                    if get_distance(sample, q) < self.max_rr*2:
                        casting_flag = True
                        break
                if casting_flag == False:
                    sample_points.append(q)
            count += 1
        return sample_points

    def generate_roadmap(self, rr, cd):
        """
 RoadMap Geneation Function
 rr: Robot Radius
 cd: Casting Distance
 @Return 2-D array Roadmap
 """
        roadmap = []
        samples = self.samples[rr]
        skdtree = self.skdtree[rr]

        nsample = len(samples)
        knn = min(int(nsample/self.knn_const), 15)

        for (i, temp_point) in zip(range(nsample), samples):
            # Query only the neighbors that can matter instead of sorting
            # all n (the old k=nsample made construction O(n^2 log n));
            # widen once if blocked neighbors exhausted the short list.
            for k in (min(nsample, 8 * knn + 1), nsample):
                index, dists = skdtree.search(temp_point, k=k)
                edge_id = []
                # 0 is self so start with 1
                for j in range(1, len(index)):
                    npoint = samples[index[j]]
                    if self.isEdgeCollisionFree(temp_point, npoint, rr) and get_distance(temp_point, npoint) < cd:
                        edge_id.append(index[j])
                    if len(edge_id) >= knn:
                        break
                if len(edge_id) >= knn or k == nsample or dists[-1] >= cd:
                    break
            roadmap.append(edge_id)
        return roadmap

    def build_task_roadmap(self, goal_samples):
        """
 Make task planning elements
 goal_samples: goal samples list
 @Update task_samples, task_roadmap
 """
        max_rr = self.max_rr
        starts = []
        goals = []
        for name, ele in self.robots_map.items():
            starts.append(ele.pos)
        for ele in goal_samples:
            goals += ele
        self.task_samples, self.task_roadmap = self.add_query(starts, goals, max_rr)

    def find_random_collision_free_configuration(self, rr):
        """
 Unifrom Sampling Function
 rr: Robot radius
 @Return free configuration with uniform sampling
 """
        while True:
            # get a random point
            q = self.get_random_point(self.bounds)
            if self.isPointCollisionFree(q, rr) and not self.isOutOfBounds(q, rr):
                return q

    def find_bridge_collision_free_configuration(self, rr):
        """
 Bridge Sampling Function
 rr: Robot Radius
 @Return free configuration with Bridge sampling
 Strategy: Find two obstacle configurations and find mid config between them.
 If that config is free config, return it.
 """
        while True:
            # get random two points
            bounds = self.bounds
            x_margin = (bounds[2] - bounds[0]) / 10
            y_margin = (bounds[3] - bounds[1]) / 10
            bounds = (bounds[0]-x_margin, bounds[1] - y_margin, bounds[2]+x_margin, bounds[3]+y_margin)
            # Bound obstacle Margin or not
            #q1 = self.get_random_point(bounds)
            #q2 = self.get_random_point(bounds)
            q1 = self.get_random_point(self.bounds)
            q2 = self.get_random_point(self.bounds)
            if ((not self.isPointCollisionFree(q1, rr)) or self.isOutOfBounds(q1, rr)) and \
                ((not self.isPointCollisionFree(q2, rr)) or self.isOutOfBounds(q2, rr)):
                q = (round((q1[0]+q2[0])/2, 2), round((q1[1]+q2[1])/2, 2))
                if self.isPointCollisionFree(q, rr) and not self.isOutOfBounds(q, rr):
                    return q

    def get_random_point(self, bounds):
        """
 Get Random Point
 @Return (x,y) inside of boundary
 """
        x = bounds[0] + random.random()*(bounds[2]-bounds[0])
        x = round(x, 2)
        y = bounds[1] + random.random()*(bounds[3]-bounds[1])
        y = round(y, 2)
        return (x,y)

    def isPointCollisionFree(self, q, rr):
        """
 Check given configuration is on collision
 q: Configuration
 rr: Robot Radius
 @Return True if there is no collision, else False
 """
        return not self._obs_prep(rr).intersects(Point(q))

    def _obs_prep(self, rr):
        """Prepared union of all obstacles inflated by rr: one C-side
 intersection test replaces a Python loop over every obstacle
 (distance(x) <= rr <=> x intersects buffer(rr))."""
        cached = self._obs_prep_cache.get(rr)
        if cached is None:
            from shapely.prepared import prep
            cached = prep(self.env.obstacle_union.buffer(rr))
            self._obs_prep_cache[rr] = cached
        return cached

    def isEdgeCollisionFree(self, q1, q2, rr):
        """
 Check given two configuration connect or not
 q1, q2: Configuration
 rr: Robot Radius
 @Return True if no obstacle between two points, else False
 """
        key = (q1, q2, rr) if q1 <= q2 else (q2, q1, rr)
        cached = self._edge_free_cache.get(key)
        if cached is not None:
            return cached
        result = not self._obs_prep(rr).intersects(LineString([q1, q2]))
        self._edge_free_cache[key] = result
        return result

    def isOutOfBounds(self, q, rr):
        """
 Check config is out of boundary
 q: Configuration
 rr: Robot Radius
 @Return True if out of boundary, else False
 """
        bounds = self.bounds
        if((q[0]-rr) < bounds[0]):
            return True
        if((q[1]-rr) < bounds[1]):
            return True
        if((q[0]+rr) > bounds[2]):
            return True
        if((q[1]+rr) > bounds[3]):
            return True
        return False


