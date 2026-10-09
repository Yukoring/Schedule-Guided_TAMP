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


class Planner(BasePDDLGenerator):
    """Our approach's generator: encodes VRP guidance into PDDL 2.2 problems.

 const selects the guidance variant: 0 normal, 1 time-bound (TILs),
 2 robot-task (+helpful route), 3 task-order, 4 restricted time-bound,
 5 tour-restricted connections, 6 time-bound + robot-task combined,
 7 sound lower-bound TILs (isJobOfType appears at its admissible earliest
 completion; computed from the world, not the oracle solution),
 8 soft robot-task (non-recommended pairs keep can_perform but pay 2x
 task_duration; completeness-preserving bias),
 9 schedule-window capability (can_perform exists only around the
 oracle's scheduled slot for the assigned robot, via TILs).
 """
    def generate_initial(self, vrp_info, const):
        """
 Make Initial State for Problem File
 Waypoint need 1. vertex free, 2. edge free, 3. connected, 4. distance
 Robot need 1. at, 2. velocity
 Job need 1. located, 2. service, 3. work free (there are many waypoints to work on job, so we limited it only at once one job)
 """
        self.pFile.write('(:init\n')
        # It's current waypoint and next waypoint
        # helpful_start = vrp_info['helpful_route'][0]
        # helpful_next = vrp_info['helpful_route'][1]

        for name, obj in self.obj_ins.items():
            for ele in obj:
                if name == 'waypoint':
                    if ele.vertex:
                        self.pFile.write('        ')
                        self.pFile.write('(vertex_free ' + ele.ind + ')\n')
                    if ele.reserved:
                        self.pFile.write('        ')
                        self.pFile.write('(reserved ' + ele.ind + ')\n')
                    # If They need Helpful Action.
                    if const == 2 or const == 6:
                        if vrp_info['robot_help']:
                            # the timed-edge hint applies only to legs along a roadmap edge
                            if (ele.ind in vrp_info['robot_help']['start']
                                    and vrp_info['robot_help']['end'][
                                        vrp_info['robot_help']['start'].index(ele.ind)]
                                    in ele.connected):
                                help_ind = vrp_info['robot_help']['start'].index(ele.ind)
                                connect_ind = ele.connected.index(vrp_info['robot_help']['end'][help_ind])
                                connect_time = vrp_info['robot_help']['time'][help_ind]
                                if connect_time <= 0:
                                    connect_time = 0
                                else:
                                    connect_time = round(connect_time/100, 2)

                                for i in range(len(ele.connected)):
                                    if i == connect_ind:
                                        self.pFile.write('        ')
                                        if connect_time == 0:
                                            # A timed initial literal must have t > 0;
                                            # an immediately-available edge is a plain fact.
                                            self.pFile.write('(connected ' + ele.ind + ' ' + vrp_info['robot_help']['end'][help_ind] + ')\n')
                                        else:
                                            self.pFile.write('(at ' + str(connect_time) + ' (connected ' + ele.ind + ' ' + vrp_info['robot_help']['end'][help_ind] + '))\n')
                                    else:
                                        self.pFile.write('        ')
                                        self.pFile.write('(at ' + str(connect_time + 2.0) + ' (connected ' + ele.ind + ' ' + ele.connected[i] + '))\n')
                                    self.pFile.write('        ')
                                    self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                                    self.pFile.write(str(ele.distance[i]) + ')\n')
                            else:
                                for i in range(len(ele.connected)):
                                    self.pFile.write('        ')
                                    self.pFile.write('(connected ' + ele.ind + ' ' + ele.connected[i] + ')\n')
                                    self.pFile.write('        ')
                                    self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                                    self.pFile.write(str(ele.distance[i]) + ')\n')
                    elif const == 5:
                        if vrp_info['robot_help'] and vrp_info['tours']:
                            if ele.ind in vrp_info['robot_help']['start']:
                                help_ind = vrp_info['robot_help']['start'].index(ele.ind)
                                connect_ind = ele.connected.index(vrp_info['robot_help']['end'][help_ind])
                                connect_time = vrp_info['robot_help']['time'][help_ind]
                                if connect_time <= 0:
                                    connect_time = 0
                                else:
                                    connect_time = round(connect_time/100, 2)

                                for i in range(len(ele.connected)):
                                    if i == connect_ind:
                                        self.pFile.write('        ')
                                        if connect_time == 0:
                                            self.pFile.write('(connected ' + ele.ind + ' ' + vrp_info['robot_help']['end'][help_ind] + ')\n')
                                        else:
                                            self.pFile.write('(at ' + str(connect_time) + ' (connected ' + ele.ind + ' ' + vrp_info['robot_help']['end'][help_ind] + '))\n')
                                    else:
                                        self.pFile.write('        ')
                                        self.pFile.write('(at ' + str(connect_time + 2.0) + ' (connected ' + ele.ind + ' ' + ele.connected[i] + '))\n')
                                    self.pFile.write('        ')
                                    self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                                    self.pFile.write(str(ele.distance[i]) + ')\n')
                            elif ele.ind in vrp_info['tours']:
                                connect_way = vrp_info['tours'][ele.ind]
                                if 'depot' in connect_way:
                                    for i in range(len(ele.connected)):
                                        self.pFile.write('        ')
                                        self.pFile.write('(connected ' + ele.ind + ' ' + ele.connected[i] + ')\n')
                                        self.pFile.write('        ')
                                        self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                                        self.pFile.write(str(ele.distance[i]) + ')\n')
                                else:
                                    for i in range(len(connect_way)):
                                        con_index = ele.connected.index(connect_way[i])
                                        self.pFile.write('        ')
                                        self.pFile.write('(connected ' + ele.ind + ' ' + connect_way[i] + ')\n')
                                        self.pFile.write('        ')
                                        self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[con_index] + ') ')
                                        self.pFile.write(str(ele.distance[con_index]) + ')\n')
                            else:
                                for i in range(len(ele.connected)):
                                    self.pFile.write('        ')
                                    self.pFile.write('(connected ' + ele.ind + ' ' + ele.connected[i] + ')\n')
                                    self.pFile.write('        ')
                                    self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                                    self.pFile.write(str(ele.distance[i]) + ')\n')
                    else:
                        for i in range(len(ele.connected)):
                            self.pFile.write('        ')
                            self.pFile.write('(connected ' + ele.ind + ' ' + ele.connected[i] + ')\n')
                            self.pFile.write('        ')
                            self.pFile.write('(= (distance ' + ele.ind + ' ' + ele.connected[i] + ') ')
                            self.pFile.write(str(ele.distance[i]) + ')\n')

                elif ele.instype.find('robot') > -1:
                    #print(ele.instype)
                    self.pFile.write('        ')
                    self.pFile.write('(robot_free ' + ele.ind + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(robot_way_token ' + ele.ind + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(at ' + ele.ind + ' ' + ele.at[0] + ')\n')
                    self.pFile.write('        ')
                    self.pFile.write('(= (velocity ' + ele.ind + ') ' + str(self.velocity) + ')\n')

                    if const == 2 or const == 6 or const == 12:
                        # constraint robot_Task
                        if vrp_info['robot_task']:
                            task_list = vrp_info['robot_task'][ele.ind]
                            for task_name in task_list:
                                self.pFile.write('        ')
                                self.pFile.write('(= (task_duration ' + ele.ind + ' ' + task_name + ') ' + str(ele.action_duration[task_name]) + ')\n')
                                self.pFile.write('        ')
                                self.pFile.write('(can_perform ' + ele.ind + ' ' + task_name + ')\n')
                    elif const == 8 and vrp_info['robot_task']:
                        # soft robot-task: bias durations instead of deleting capability
                        task_list = vrp_info['robot_task'].get(ele.ind, [])
                        for task_name, action_cost in ele.action_duration.items():
                            cost = action_cost if task_name in task_list else action_cost * 2.0
                            self.pFile.write('        ')
                            self.pFile.write('(= (task_duration ' + ele.ind + ' ' + task_name + ') ' + str(cost) + ')\n')
                            self.pFile.write('        ')
                            self.pFile.write('(can_perform ' + ele.ind + ' ' + task_name + ')\n')
                    elif const == 9 and vrp_info.get('task_windows'):
                        # schedule-window capability: can_perform only around the slot
                        for task_name, action_cost in ele.action_duration.items():
                            self.pFile.write('        ')
                            self.pFile.write('(= (task_duration ' + ele.ind + ' ' + task_name + ') ' + str(action_cost) + ')\n')
                        for (r_ind, task_name), (w1, w2) in vrp_info['task_windows'].items():
                            if r_ind != ele.ind:
                                continue
                            if w1 <= 0.1:
                                self.pFile.write('        ')
                                self.pFile.write('(can_perform ' + ele.ind + ' ' + task_name + ')\n')
                            else:
                                self.pFile.write('        ')
                                self.pFile.write('(at ' + str(round(w1, 2)) + ' (can_perform ' + ele.ind + ' ' + task_name + '))\n')
                            self.pFile.write('        ')
                            self.pFile.write('(at ' + str(round(w2, 2)) + ' (not (can_perform ' + ele.ind + ' ' + task_name + ')))\n')
                    else:
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
                    task_order = vrp_info['goal_order'].get(ele.ind, [])
                    #task_order = vrp_info[ele.ind]
                    if const == 3:
                        if len(task_order) >1:
                            for i in range(len(task_order)-1):
                                self.pFile.write('        ')
                                self.pFile.write('(before ' + ele.ind + ' ' + task_order[i] + ' ' + task_order[i+1] + ')\n')

                elif name == 'task':
                    goal_locs = ele.loc
                    for goal_loc in goal_locs:
                        t_lb = self._task_lower_bound(goal_loc, ele.ind) if const == 7 else 0
                        if t_lb > 0.1:
                            self.pFile.write('        ')
                            self.pFile.write('(at ' + str(round(t_lb, 2)) + ' (isJobOfType ' + goal_loc + ' ' + ele.ind + '))\n')
                        else:
                            self.pFile.write('        ')
                            self.pFile.write('(isJobOfType ' + goal_loc + ' ' + ele.ind + ')\n')
                self.pFile.write('\n')

        if const == 1 or const == 4 or const == 6:
            time_bound = vrp_info['goal_bound']
            # (at 3.0 (not (isJobOfType goal0000 taska)))
            for name, ele in time_bound.items():
                goal_name = name
                for i, ele2 in enumerate(ele):
                    task_name = ele2[0]
                    max_bound = ele2[1]/100
                    if const == 4:
                        if ele2[2] == 2: # 0
                            self.pFile.write('        ')
                            self.pFile.write('(at ' + str(max_bound) + ' (not (isJobOfType ' + goal_name + ' ' + task_name + ')))\n')
                    else:
                        self.pFile.write('        ')
                        self.pFile.write('(at ' + str(max_bound) + ' (not (isJobOfType ' + goal_name + ' ' + task_name + ')))\n')
                self.pFile.write('\n')

        self.pFile.write(')\n')

    def _task_lower_bound(self, goal_ind, task_ind):
        """Admissible earliest completion of (goal, task): the fastest capable
 robot travelling straight from its start. Independent of any oracle
 solution, so pruning with it preserves optimality."""
        job = next(j for j in self.obj_ins['job'] if j.ind == goal_ind)
        dist_of = {w.ind: dict(zip(w.connected, w.distance))
                   for w in self.obj_ins['waypoint']}
        best = None
        for robot in self.obj_ins['robot']:
            if task_ind not in robot.action_duration:
                continue
            for wp in job.located:
                d = 0.0 if robot.at[0] == wp else dist_of.get(robot.at[0], {}).get(wp)
                if d is None:
                    continue
                t = d / self.velocity + robot.action_duration[task_ind]
                if best is None or t < best:
                    best = t
        return best or 0

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
        for name, obj in self.obj_ins.items():
            for ele in obj:
                if name.find('robot') > -1:
                    self.pFile.write('        ')
                    self.pFile.write('(robot_way_token ' + ele.ind + ')\n')
        self.pFile.write('        )\n')
        self.pFile.write(')\n')

    def generate_constraints(self, vrp_info, const):
        """const 10: the relaxation's deadlines as soft PDDL3 preferences.
 A hard TIL kills the whole encoding when the relaxation was
 optimistic (no congestion model); (within T ...) preferences let
 OPTIC trade lateness against makespan instead of dying."""
        self._pref_names = []
        if const != 10:
            return
        time_bound = vrp_info.get('goal_bound', {})
        if not time_bound:
            return
        self.pFile.write('(:constraints (and\n')
        n = 0
        for goal_name, ele in time_bound.items():
            for ele2 in ele:
                pref = 'df%d' % n
                n += 1
                self._pref_names.append(pref)
                self.pFile.write('        (preference %s (within %s '
                                 '(finish %s %s)))\n'
                                 % (pref, ele2[1] / 100, goal_name, ele2[0]))
        self.pFile.write('))\n')

    def generate_metric(self, const=0):
        """
 Metric func for minimizing something
 """
        if const == 10 and self._pref_names:
            terms = ' '.join('(* 10 (is-violated %s))' % p
                             for p in self._pref_names)
            self.pFile.write('(:metric minimize (+ (total-time) %s))\n'
                             % terms)
        else:
            self.pFile.write('(:metric minimize (total-time))\n')

    def generate_problem(self, vrp_info, count = 0, consts = None):
        # one problem file per guidance variant: U (no guidance), AD (all-task deadlines), PD (partial deadlines), RT (robot task types)
        if consts is None:
            consts = [0, 1, 2, 4]
        labels = {0: 'U', 1: 'AD', 4: 'PD', 2: 'RT'}
        stem = self.pFilename.replace('.pddl', '')
        pName = {}
        for i in consts:
            file_name = 'pddl_prob_plan/%s_round%d_%s.pddl' % (stem, count, labels.get(i, 'c%d' % i))
            pName[i] = file_name
            self.problemName = Path(file_name).stem
            self.pFile = open(file_name, "w", buffering=1)
            #print("file open for writing problem")
            self.generate_header()
            self.generate_initial(vrp_info, i)
            self.generate_goals()
            self.generate_constraints(vrp_info, i)
            self.generate_metric(i)
            self.pFile.write(')\n')
            self.pFile.close()
        return pName

