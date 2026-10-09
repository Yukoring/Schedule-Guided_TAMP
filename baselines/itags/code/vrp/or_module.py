from functools import partial
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from copy import deepcopy
from common.utils import get_sum_of_cost

# Wall-clock budget per OR-Tools solve. The makespan-tightening loop in
# solver() re-solves with a shrinking bound until infeasible, so total VRP
# time is roughly (improvements + failed proofs) * this budget.
VRP_TIME_LIMIT = 5  # seconds; 250 was used for narrow domains

class ORToolsModule:
    """
    Vehicle Routing Problem Planner Generator
    """
    def __init__(self, vrp_instance, margin_scale=1.0):  # scale unused (routing)
        self.debug = vrp_instance.env.debug_mode
        self.obj_ins = vrp_instance.obj_ins
        self.waypoints = vrp_instance.obj_ins['waypoint']
        self.robots = vrp_instance.obj_ins['robot'] # ele.ind, ele.at
        self.jobs = vrp_instance.obj_ins['job']     # ele.ind, ele.located, ele.order
        self.tasks = vrp_instance.obj_ins['task']   # ele.ind, ele.loc
        self.tokens = vrp_instance.obj_ins.get('token',[])

        # distance matrix is {'wp0': {'wp1': 6.31, 'wp2': 9.12 ...}, ...}
        self.distance_matrix = dict()
        self.path_matrix = dict()
        self.waypoint_dict = {}

        self.max_time = None

        self.useless_way = []

        for token_set in self.tokens:
            if token_set.token_name == 'normal_col_token':
                self.useless_way += token_set.token_wp

        for ele in self.waypoints:
            temp_dist = dict()
            temp_path = dict()
            for index, ele2 in enumerate(ele.connected):
                temp_dist[ele2] = ele.distance[index]
                temp_path[ele2] = ele.path[index]
            self.distance_matrix[ele.ind] = temp_dist
            self.path_matrix[ele.ind] = temp_path

    class Way_ins:
        def __init__(self, root_name):
            self.root = root_name
            self.goal = None
            self.task = None
            self.node_index = None
            self.task_duration = dict()
            self.loc = None

    def create_data_model(self):
        """Stores the data for the problem."""
        data = {}
        robot_num = len(self.robots)

        data['num_vehicles'] = robot_num
        data['depot'] = 0
        data['distance_matrix'] = []
        data['way_name'] = ['depot']

        data['service_time'] = []
        data['speed'] =[]
        data['starts'] = []
        data['ends'] = []
        data['orders'] = []

        data['node_group'] = []
        data['task_group'] = []
        data['col_group'] = []
        data['token'] = []

        # Organize Waypoint
        node_ind = 1
        for root_way in self.waypoints:
            waypoint = self.Way_ins(root_way.ind)
            node_number = 0
            waypoint.loc = root_way.loc
            name = root_way.ind + '-'

            # Robot Location Check
            for robot in self.robots:
                if root_way.ind == robot.at[0]:
                    temp_waypoint = deepcopy(waypoint)
                    temp_name = name + str(node_number)
                    temp_waypoint.node_index = node_ind
                    data['starts'].append(node_ind)
                    self.waypoint_dict[temp_name] = temp_waypoint
                    node_ind += 1
                    node_number += 1

            # Goal - Task Check
            # First waypoint have the job, Second Job include the task
            for job in self.jobs:
                if root_way.ind in job.located:
                    temp_nodes = []
                    for task in self.tasks:
                        if job.ind in task.loc:
                            temp_waypoint = deepcopy(waypoint)
                            temp_name = name + str(node_number)
                            temp_waypoint.node_index = node_ind
                            temp_nodes.append(node_ind)
                            temp_waypoint.goal = job.ind
                            temp_waypoint.task = task.ind
                            for robot in self.robots:
                                if temp_waypoint.task in robot.action_duration:
                                    temp_waypoint.task_duration[robot.ind] = robot.action_duration[temp_waypoint.task]
                            self.waypoint_dict[temp_name] = temp_waypoint
                            node_ind += 1
                            node_number += 1
                    data['node_group'].append(temp_nodes)
                    data['task_group'].append(temp_nodes)

            # Collision Waypoint Check
            if node_number == 0:
                if not root_way.ind in self.useless_way:
                    temp_nodes = []
                    for i in range(robot_num):
                        temp_waypoint = deepcopy(waypoint)
                        temp_name = name + str(node_number)
                        temp_waypoint.node_index = node_ind
                        temp_nodes.append(node_ind)
                        self.waypoint_dict[temp_name] = temp_waypoint
                        node_ind += 1
                        node_number += 1
                    data['node_group'].append(temp_nodes)
                    data['col_group'].append(temp_nodes)

        # for name, way in self.waypoint_dict.items():
        #     print(name, way.root, way.goal, way.task)
        #     print(way.loc, way.node_index)

        # distance matrix
        depot_dist = [0 for i in range(node_ind)]
        data['distance_matrix'].append(depot_dist)

        for name1, way1 in self.waypoint_dict.items():
            root1 = way1.root
            temp_dist = [0] # For depot
            for name2, way2 in self.waypoint_dict.items():
                root2 = way2.root
                dist = self.distance_matrix[root1].get(root2, -1)
                if root1 == root2:
                    temp_dist.append(0)
                elif dist != -1:
                    temp_dist.append(int(dist*100))
                else:
                    temp_dist.append(10000000)
            data['distance_matrix'].append(temp_dist)
            data['way_name'].append(name1)

        # Listing for robot related (including Token)
        for i in range(robot_num):
            service = [0] # init for depot first
            robot_task = self.robots[i].action_duration
            for name, ele in self.waypoint_dict.items():
                if ele.task:
                    if ele.task in robot_task:
                        service.append(int(robot_task[ele.task] * 100))
                        # service.append(int(robot_task[ele.task] * 25))
                    else:
                        service.append(10000000)
                else:
                    service.append(0)
            data['service_time'].append(service)
            data['speed'].append(1)
            data['ends'].append(0)
            data['token'].append(len(data['distance_matrix']))

        # Order Constraints
        for job in self.jobs:
            for order in job.order:
                temp_order = [None,None]
                for name, way in self.waypoint_dict.items():
                    if job.ind == way.goal and order[0] == way.task:
                        temp_order[0] = way.node_index
                    if job.ind == way.goal and order[1] == way.task:
                        temp_order[1] = way.node_index
                data['orders'].append(temp_order)

        # Grouping for same goal
        data['goal_group'] = []
        for task in self.tasks:
            for goal in task.loc:
                temp_group = []
                for name, way in self.waypoint_dict.items():
                    if way.goal == goal and way.task == task.ind:
                        temp_group.append(way.node_index)
                data['goal_group'].append(temp_group)
        return data

    def parse_vrp_plan(self, data, manager, routing, solution):
        if self.debug:
            print(f'Objective: {solution.ObjectiveValue()}')
        time_dimension = routing.GetDimensionOrDie('Time')
        total_time = 0

        path = dict()

        vrp_info = dict()
        vrp_info['helpful_route'] = []
        temp_start_wp = []
        temp_next_wp = []

        max_makespan = 0

        task_order = dict()

        robot_task = dict()
        goal_order = dict()
        goal_bound = dict()
        robot_help = dict()
        robot_help['start'] = []
        robot_help['end'] = []
        robot_help['time'] = []
        tours = dict()

        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            robot_instance = self.robots[vehicle_id]
            robot_name = robot_instance.ind
            plan_output = 'Route for wehicle {}:\n'.format(vehicle_id)
            tour_count = 0
            path[robot_name] = []
            robot_task[robot_name] = []
            prev_dist = 0
            curr_time = 0
            curr_way = data['way_name'][manager.IndexToNode(index)]

            # For Helpful route.
            helpful_flag = 0
            # For robot task
            task_list = []

            if self.waypoint_dict[curr_way].task and \
                self.waypoint_dict[curr_way].task in robot_instance.action_duration:
                temp_time = robot_instance.action_duration[self.waypoint_dict[curr_way].task]
                temp_task = self.waypoint_dict[curr_way].task
                temp_goal = self.waypoint_dict[curr_way].goal
                temp_path = [self.waypoint_dict[curr_way].loc]
                temp_list = [temp_path, curr_time, temp_time, temp_task, temp_goal]
                path[robot_name].append(temp_list)
                if not temp_task in task_list:
                    robot_task[robot_name].append(temp_task)
                    task_list.append(temp_task)
                curr_time += temp_time

            while not routing.IsEnd(index):
                curr_way = data['way_name'][manager.IndexToNode(index)]
                next_way = data['way_name'][manager.IndexToNode(solution.Value(routing.NextVar(index)))]

                time_var = time_dimension.CumulVar(index)
                if self.waypoint_dict[curr_way].task:
                    max_solution = 0
                    col_way = 0
                    if solution.Min(time_var) != solution.Max(time_var):
                        max_solution = solution.Max(time_var)
                    if self.waypoint_dict[curr_way].root in self.useless_way:
                        col_way = 1
                    task_order[self.waypoint_dict[curr_way].goal] = task_order.get(self.waypoint_dict[curr_way].goal, []) \
                                     + [(solution.Min(time_var),self.waypoint_dict[curr_way].task,
                                     int(robot_instance.action_duration[self.waypoint_dict[curr_way].task] * 100),
                                     tour_count, vehicle_id, max_solution, col_way)]
                    tour_count += 1
                    if next_way != 'depot':
                        curr_root = self.waypoint_dict[curr_way].root
                        next_root = self.waypoint_dict[next_way].root
                        if curr_root != next_root:
                            if not next_root in tours.get(curr_root,[]):
                                tours[curr_root]  = tours.get(curr_root, []) + [next_root]
                    else:
                        curr_root = self.waypoint_dict[curr_way].root
                        if not 'depot' in tours.get(curr_root,[]):
                            tours[curr_root]  = tours.get(curr_root, []) + ['depot']


                if helpful_flag == 1:
                    if self.waypoint_dict[curr_way].task:
                        robot_help['time'].append((solution.Min(time_var) - prev_dist -
                                                  int(robot_instance.action_duration[self.waypoint_dict[curr_way].task]*100)))
                    else:
                        robot_help['time'].append(solution.Min(time_var) - prev_dist)
                    helpful_flag += 1

                if next_way != 'depot':
                    curr_root = self.waypoint_dict[curr_way].root
                    next_root = self.waypoint_dict[next_way].root
                    if curr_root != next_root:
                        temp_path1 = self.path_matrix[curr_root][next_root]
                        temp_time1 = self.distance_matrix[curr_root][next_root]
                        temp_list1 = [temp_path1, curr_time, temp_time1]
                        path[robot_name].append(temp_list1)
                        curr_time += temp_time1
                        if helpful_flag == 0:
                            temp_start_wp.append(curr_root)
                            temp_next_wp.append(next_root)
                            helpful_flag += 1
                            robot_help['start'].append(curr_root)
                            robot_help['end'].append(next_root)
                            prev_dist = int(temp_time1*100)

                    if self.waypoint_dict[next_way].task and \
                        self.waypoint_dict[next_way].task in robot_instance.action_duration:
                        temp_time2 = robot_instance.action_duration[self.waypoint_dict[next_way].task]
                        temp_task = self.waypoint_dict[next_way].task
                        temp_goal = self.waypoint_dict[next_way].goal
                        temp_path2 = [self.waypoint_dict[next_way].loc]
                        temp_list2 = [temp_path2, curr_time, temp_time2, temp_task, temp_goal]
                        path[robot_name].append(temp_list2)
                        if not temp_task in task_list:
                            robot_task[robot_name].append(temp_task)
                            task_list.append(temp_task)
                        curr_time += temp_time2
                else:
                    helpful_flag = True
                #print(temp_start_wp, temp_next_wp)

                if temp_start_wp:
                    vrp_info['helpful_route'] = [temp_start_wp, temp_next_wp]

                plan_output += '{0} Time({1},{2}) -> '.format(
                    data['way_name'][manager.IndexToNode(index)], solution.Min(time_var),
                    solution.Max(time_var))
                index = solution.Value(routing.NextVar(index))

            time_var = time_dimension.CumulVar(index)
            max_makespan = max(max_makespan, curr_time)
            plan_output += '{0} Time({1},{2})\n'.format(data['way_name'][manager.IndexToNode(index)],
                                                        solution.Min(time_var),
                                                        solution.Max(time_var))
            plan_output += 'Time of the route: {}min\n'.format(solution.Min(time_var))
            if self.debug:
                print(plan_output)
            total_time += solution.Min(time_var)

        if self.debug:
            print('Total time of all routes: {}min'.format(total_time))

        # For Task Order
        timeline = []
        for name, ele in task_order.items():
            ele.sort()
            temp_task = []
            for ele2 in ele:
                temp_task.append(ele2[1])
                timeline.append([ele2[0],name,ele2[1],ele2[2],ele2[3],ele2[4],ele2[6]])
            goal_order[name] = temp_task
        vrp_info['goal_order'] = goal_order
        if self.debug:
            print(task_order)
        timeline.sort()

        # Timeline Based Post Processing
        # Goal Bound
        # Check for Occupyin same time.
        for i in range(len(timeline)-1):
            for j in range(i+1, len(timeline)):
                curr_ele = timeline[i]
                next_ele = timeline[j]
                curr_t = curr_ele[0]
                next_t = next_ele[0]
                next_arrival = next_t - next_ele[3]
                time_delay = 50
                if next_arrival < curr_t + time_delay:
                    if curr_ele[1] == next_ele[1]:
                        add_robot = next_ele[5]
                        curr_ele[6] = 1
                        next_ele[6] = 1
                        add_time = curr_t + time_delay - next_arrival
                        for k in range(j, len(timeline)):
                            if timeline[k][5] == add_robot:
                                timeline[k][0] += add_time
                else:
                    break
        for ele in timeline:
            time_dealy = 50 + ele[4] * 50 # 25, 50
            if ele[4] == 0:
                ele[6] = 2
            goal_bound[ele[1]] = goal_bound.get(ele[1],[]) + [(ele[2],ele[0]+time_dealy, ele[6])]

        vrp_info['goal_bound'] = goal_bound
        vrp_info['robot_task']= robot_task
        # capability windows from the timeline: [arrival, completion+margin]
        task_windows = dict()
        for ele in timeline:
            r_ind = self.robots[ele[5]].ind
            slack = (50 + ele[4] * 50) / 100
            w1 = max(0.0, (ele[0] - ele[3]) / 100 - slack)
            w2 = ele[0] / 100 + slack
            key = (r_ind, ele[2])
            if key in task_windows:
                w1 = min(w1, task_windows[key][0])
                w2 = max(w2, task_windows[key][1])
            task_windows[key] = (w1, w2)
        vrp_info['task_windows'] = task_windows
        vrp_info['robot_help'] = robot_help
        vrp_info['tours'] = tours

        return path, max_makespan, vrp_info

    def solver(self):
        """Solve the VRP with time windows."""
        # Instantiate the data problem.
        data = self.create_data_model()

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                            data['num_vehicles'], data['starts'], data['ends'])


        # If we use the distance matrix, comment out this.

        # def distance_callback(from_index, to_index):
        #     """Returns the travel time between the two nodes."""
        #     # Convert from routing variable Index to time matrix NodeIndex.
        #     from_node = manager.IndexToNode(from_index)
        #     to_node = manager.IndexToNode(to_index)
        #     return data['distance_matrix'][from_node][to_node]

        # transit_dist_callback = routing.RegisterTransitCallback(distance_callback)
        # routing.SetArcCostEvaluatorOfAllVehicles(transit_dist_callback)

        # # Create and register a transit callback.
        def time_callback(from_index, to_index, ind):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(data['distance_matrix'][from_node][to_node]/data['speed'][ind]) + \
                data['service_time'][ind][to_node]

        def token_visiting_callback(from_index, to_index):
            from_node = manager.NodeToIndex(from_index)
            to_node = manager.NodeToIndex(to_index)
            return 1

        # For makespan not for sum fo cost
        vehicle_max = 100000
        path = None
        makespan = -1
        vrp_info = dict()
        vrp_info['helpful_route'] = []
        vrp_info['goal_order'] = dict()
        vrp_info['goal_bound'] = dict()
        vrp_info['robot_task'] = dict()
        vrp_info['robot_help'] = dict()
        vrp_info['tours'] = dict()
        vrp_info['task_windows'] = dict()

        while True:
            # Create Routing Model.
            routing = pywrapcp.RoutingModel(manager)

            transit_time_callback = []

            for v in range(manager.GetNumberOfVehicles()):
                transit_callback_partial = partial(time_callback, ind = v)
                transit_time_callback.append(routing.RegisterTransitCallback(transit_callback_partial))
                routing.SetArcCostEvaluatorOfVehicle(transit_time_callback[v], v)
            # routing.SetArcCostEvaluatorOfAllVehicles(max(transit_time_callback))


            time = 'Time'
            routing.AddDimensionWithVehicleTransitAndCapacity(
                transit_time_callback,
                0,  # no slack
                [vehicle_max for i in range(data['num_vehicles'])],  # vehicle maximum travel time
                True,  # start cumul to zero
                time)

            time_dimension = routing.GetDimensionOrDie(time)
            # time_dimension.SetGlobalSpanCostCoefficient(100)


            token_callback_index = routing.RegisterTransitCallback(token_visiting_callback)
            routing.AddDimensionWithVehicleCapacity(
                token_callback_index,
                0,
                data['token'],
                True,
                'Token'
            )
            token_dimension = routing.GetDimensionOrDie('Token')

            # for i in range(data['num_vehicles']):
            #     routing.AddVariableMinimizedByFinalizer(
            #         time_dimension.CumulVar(routing.Start(i)))
            #     routing.AddVariableMinimizedByFinalizer(
            #         time_dimension.CumulVar(routing.End(i)))

            # Add constraints (Order)
            for order in data['orders']:
                task1 = manager.NodeToIndex(order[0])
                task2 = manager.NodeToIndex(order[1])
                routing.solver().Add(time_dimension.CumulVar(task1) + 120 <
                                    time_dimension.CumulVar(task2))

            # Add Disjunction for goal
            for name, way in self.waypoint_dict.items():
                if not way.goal:
                    routing.AddDisjunction([manager.NodeToIndex(way.node_index)], 0)

            for group in data['goal_group']:
                disjunction = []
                for ele in group:
                    disjunction.append(manager.NodeToIndex(ele))
                routing.AddDisjunction(disjunction, -1)

            if self.debug:
                print(data['goal_group'])
                print(data['task_group'])

            for group in data['task_group']:
                if len(group) > 1:
                    for i in range(len(group)-1):
                        task1 = manager.NodeToIndex(group[i])
                        for j in range(i+1, len(group)):
                            task2 = manager.NodeToIndex(group[j])
                            routing.solver().Add(token_dimension.CumulVar(task1) != token_dimension.CumulVar(task2))


            # Setting first solution heuristic.
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            # search_parameters.first_solution_strategy = (
            #     routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
            search_parameters.first_solution_strategy = (
                  routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

            search_parameters.local_search_metaheuristic = (
                routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
            search_parameters.time_limit.seconds = VRP_TIME_LIMIT

            # Solve the problem.
            solution = routing.SolveWithParameters(search_parameters)

            # Print solution on console.

            if solution:
                path, makespan, vrp_info = self.parse_vrp_plan(data, manager, routing, solution)
                vehicle_max = max(0, int(makespan*100 - 10))
            else:
                break
        if not path:
            print('No solution found !')


        return path, makespan, vrp_info

