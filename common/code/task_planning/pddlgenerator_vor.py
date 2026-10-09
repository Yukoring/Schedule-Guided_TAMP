import copy
import math
import time
from math import degrees
from pathlib import Path

import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString

from common.utils import get_sum_of_cost, KDTree, get_distance
from common.visualize import *
from task_planning.base_generator import BasePDDLGenerator, read_domain


class PDDLProblemGenerator(BasePDDLGenerator):
    """Baseline generator (CBTAMP / prioritized / planner-only): plain
 problem encoding, minimizing total-time."""
    def generate_initial(self):
        """
 Make Initial State for Problem File
 Waypoint need 1. vertex free, 2. edge free, 3. connected, 4. distance
 Robot need 1. at, 2. velocity
 Job need 1. located, 2. service, 3. work free (there are many waypoints to work on job, so we limited it only at once one job)
 """
        self.pFile.write('(:init\n')

        for name, obj in self.obj_ins.items():
            for ele in obj:
                if name == 'waypoint':
                    if ele.vertex:
                        self.pFile.write('        ')
                        self.pFile.write('(vertex_free ' + ele.ind + ')\n')
                    if ele.reserved:
                        self.pFile.write('        ')
                        self.pFile.write('(reserved ' + ele.ind + ')\n')
                    for i in range(len(ele.connected)):
                        self.pFile.write('        ')
                        self.pFile.write('(connected ' + ele.ind + ' ' + ele.connected[i] + ')\n')
                        self.pFile.write('        ')
                        self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                        self.pFile.write(str(ele.distance[i]) + ')\n')
                elif ele.instype.find('robot') > -1:
                    #print(ele.instype)
                    self.pFile.write('        ')
                    self.pFile.write('(at ' + ele.ind + ' ' + ele.at[0] + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(= (velocity ' + ele.ind + ') ' + str(self.velocity) + ')\n')

                    for task_name, action_cost in ele.action_duration.items():
                        self.pFile.write('        ')
                        self.pFile.write('(= (task_duration ' + ele.ind + ' ' + task_name + ') ' + str(action_cost) + ')\n')
                        self.pFile.write('        ')
                        self.pFile.write('(can_perform ' + ele.ind + ' ' + task_name + ')\n')

                elif ele.instype == 'job':
                    for wp in ele.located:
                        self.pFile.write('        ')
                        self.pFile.write('(located ' + ele.ind + ' ' + wp + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(work_free ' + ele.ind + ')\n')
                    for order in ele.order:
                        self.pFile.write('        ')
                        self.pFile.write('(before ' + ele.ind + ' ' + order[0] + ' ' + order[1] + ')\n')
                elif name == 'task':
                    goal_locs = ele.loc
                    for goal_loc in goal_locs:
                        self.pFile.write('        ')
                        self.pFile.write('(isJobOfType ' + goal_loc + ' ' + ele.ind + ')\n')

                self.pFile.write('\n')

        self.pFile.write(')\n')

    def generate_goals(self):
        """
 Generate Goal for Problem File
 now only service needed for it.
 """
        self.pFile.write('(:goal (and\n')
        for task in self.obj_ins['task']:
            for ele in task.loc:
                self.pFile.write('        ')
                self.pFile.write('(finish ' + ele + ' ' + task.ind + ')\n')
        self.pFile.write('        )\n')
        self.pFile.write(')\n')

    def generate_metric(self):
        """
 Metric func for minimizing something
 """
        self.pFile.write('(:metric minimize (total-time))\n')

    def generate_problem(self, count=0):
        pName = 'pddl_prob_plan/%s_round%d.pddl' % (self.pFilename.replace('.pddl', ''), count)
        self.problemName = Path(pName).stem
        self.pFile = open(pName, "w", buffering=1)
        self.generate_header()
        self.generate_initial()
        self.generate_goals()
        self.generate_metric()
        self.pFile.write(')\n')
        self.pFile.close()
        #print("file close")
        #print("Problem Generated!!!")
        return pName

    def parse_plan(self, planFile):
        """
 Plan parse for prioritized planning
 output is agent - waypoint - action set
 """
        f = Path(planFile)
        if not f.is_file():
            raise BaseException(planFile + " does not exists.")
        f = open(planFile, 'r')
        lines = f.readlines()
        plan_list = dict()
        #robot_dict = dict
        way_dict = dict()
        cost = 0
        for r in self.obj_ins['robot']:
            plan_list[r.ind] = []
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
                # add the line to every robot whose name appears in it
                for r in plan_list:
                    if r in line:
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
                        if wp1 in way_dict:
                            wp1 = way_dict[wp1]['loc']
                        if wp2 in way_dict:
                            wp2 = way_dict[wp2]['loc']
                        plan_list[r].append([wp1, wp2])

            if 'do_task_single' in line:
                for r in plan_list:
                    if r in line:
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
                        plan_list[r].append([wp1, action_d, task_name, goal_name])

            if 'do_task_double' in line:
                for r in plan_list:
                    if r in line:
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
                        plan_list[r].append([wp1, action_d, task_name, goal_name])
        f.close()

        return plan_list

