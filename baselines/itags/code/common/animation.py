#!/usr/bin/env python3
"""Final-plan animation (mp4). Extracted verbatim from visualize.py;
colors now come from common.style so robot i matches every figure."""
import math
from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, transforms
from matplotlib.patches import Circle, Polygon, Rectangle
from shapely.geometry import LineString, Point

from common.style import ROBOT_COLORS as Robot_Colors
from common.style import C as _C
from common.style import shapely_patch
from common.utils import *

Colors = Robot_Colors            # path color == robot color
Goal_Colors = ['white'] * 10     # goal samples: neutral, world-styled
Task_Colors = ['royalblue', 'gold', 'limegreen', 'darkgray', 'brown']


def PolygonPatch(geom, fc='gray', ec='black', alpha=1.0, zorder=1):
    if fc == 'blue':             # legacy obstacle color -> themed
        fc, ec = _C['obstacle'], _C['obstacle_e']
    elif fc == 'green':          # legacy goal region color -> themed
        fc, ec = _C['goal_face'], _C['goal_edge']
    return shapely_patch(geom, fc, ec, alpha=alpha, zorder=zorder)


class Animation:
    def __init__(self, env, obj_ins, goals, robot_path, v_text):
        self.env = env
        self.obj_ins = obj_ins
        self.goals = goals
        self.robots = env.robots_map
        self.robot_path = robot_path
        self.goals_map = env.goals_map
        # print(robot_path)


        # figure size
        minx, miny, maxx, maxy = env.bounds
        # max_instance = max(self.env.num_of_robot, self.env.num_of_goal)
        y_bound = maxy + 1.5
        # y_bound = 0.7*(max_instance) + maxy - 1.0

        aspect = (maxx+2)/y_bound

        #self.fig = plt.figure(frameon=False, figsize=figsize)
        self.fig = plt.figure(frameon=False, figsize = (8 * aspect, 8.25)) #8
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        self.agent_service = dict()
        self.agent_service_order = dict()
        self.task_patches = dict()
        self.goal_names = dict()

        # label
        self.label_agent_names = dict()
        # self.label_agent_type = dict()
        # self.label_goal_names = dict()
        self.label_goal_service = dict()
        self.time_text = None
        self.total_cost_text = None
        self.makespan_text = None

        self.path_with_time = dict()

        self.T = 0

        plt.xlim([minx-1, maxx+1])
        plt.ylim([miny, y_bound])
        # plt.title(" Animation")
        # Add Obstacles and Bound
        self.patches.append(Rectangle((minx, miny), maxx - minx, maxy - miny, facecolor='none', edgecolor='gray'))
        for name, obs in env.obstacles_map.items():
            self.patches.append(PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20))

        # Add Goal instance
        # Square, Star, Circle, Triangle, diamond
        total_task = []
        for name,goal in self.goals_map.items():
            for service in goal['service']:
                if not service in total_task:
                    total_task.append(service)

        for i, (name, goal) in enumerate(self.goals_map.items()):
            self.task_patches[name] = dict()
            # Goal Region and Task
            if goal['based'] == 'region':
                self.patches.append(PolygonPatch(goal['shape'], fc='green', ec='black', alpha=1.0, zorder=1))
            # Task instance- order ?
            bminx, bminy, bmaxx, bmaxy = goal['shape'].bounds
            task_ind = [[0,ele] for ele in goal['service']]
            temp_ord = deepcopy(goal['order'])
            while len(temp_ord) > 0:
                temp_name = []
                temp_remove = []
                for ele in temp_ord:
                    if not ele[0] in temp_name:
                        temp_name.append(ele[0])
                    if not ele[1] in temp_name:
                        temp_name.append(ele[1])
                for ele in temp_ord:
                    if ele[1] in temp_name:
                        temp_name.remove(ele[1])
                        for ele2 in task_ind:
                            if ele2[1] == ele[1]:
                                ele2[0] += 1
                for ele in temp_ord:
                    for ele2 in temp_name:
                        if ele2 in ele:
                            temp_remove.append(ele)
                for ele in temp_remove:
                    if ele in temp_ord:
                        temp_ord.remove(ele)
            #
            prev_ind = 0
            std_x = bmaxx + 0.3
            std_y = bmaxy + 0.3
            task_ind.sort()
            for ele in task_ind:
                if prev_ind == ele[0]:
                    std_y -= 0.3
                else:
                    std_x += 0.3
                    std_y = bmaxy
                    prev_ind = ele[0]
                # only 5 marker shapes exist; cycle for maps with more tasks
                task_shape = total_task.index(ele[1]) % 5
                if task_shape == 0: #Square
                    self.task_patches[name][ele[1]] = Rectangle((std_x-0.1, std_y-0.1), 0.2, 0.2, fc='royalblue', ec='black')
                elif task_shape == 1: #Star
                    sx = [std_x,std_x-0.025,std_x-0.1,std_x-0.025,std_x,std_x+0.025,std_x+0.1,std_x+0.025]
                    sy = [std_y+0.1,std_y+0.025,std_y,std_y-0.025,std_y-0.1,std_y-0.025,std_y,std_y+0.025]
                    self.task_patches[name][ele[1]] = Polygon(xy=list(zip(sx,sy)), fc='yellow', ec='black')
                elif task_shape == 2: #Circle
                    self.task_patches[name][ele[1]] = Circle((std_x,std_y), 0.1, fc='limegreen', ec='black')
                elif task_shape == 3: #Triangle
                    sx = [std_x, std_x-0.1, std_x+0.1]
                    sy = [std_y+0.13, std_y-0.07, std_y-0.07]
                    self.task_patches[name][ele[1]] = Polygon(xy=list(zip(sx,sy)), fc='darkgray', ec='black')
                elif task_shape == 4: #Diamond
                    sx = [std_x, std_x-0.05, std_x, std_x+0.05]
                    sy = [std_y+0.1, std_y, std_y - 0.1, std_y]
                    self.task_patches[name][ele[1]] = Polygon(xy=list(zip(sx,sy)), fc='brown', ec='black')
                self.patches.append(self.task_patches[name][ele[1]])

            for goal in self.goals[i]:
                buffered_goal_sample = Point(goal).buffer(0.10)
                self.patches.append(PolygonPatch(buffered_goal_sample, fc = Goal_Colors[i % len(Goal_Colors)], ec='black', alpha=1.0, zorder=1.0))

        # Add Path
        # for name, ele in self.robots.items():
        #     ind = ele.index
        #     local_path = ele.path['path_only']
        #     if len(local_path) > 1:
        #         buffered_line = LineString(local_path).buffer(0.03)
        #         line_color = Colors[ind % len(Colors)]
        #         self.patches.append(PolygonPatch(buffered_line, fc = line_color, ec= line_color, alpha=1.0, zorder=1.0))

        # Add Agent
        for name, ele in self.robots.items():
            self.agent_service[name] = dict()
            self.agent_service_order[name] = []
            self.agents[name] = Circle((ele.pos[0], ele.pos[1]), ele.robot_radius, facecolor=Robot_Colors[i % len(Robot_Colors)],
                                    edgecolor='black')
            self.agents[name].original_face_color = Robot_Colors[ele.index % len(Robot_Colors)]
            self.patches.append(self.agents[name])
            self.T = max(self.T, self.robots[name].path_time)

            self.agent_names[name] = self.ax.text(ele.pos[0], ele.pos[1] + 0.4, name, fontsize = 10)
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

            temp_agent_task = []
            for i in range(len(ele.service)):
                temp_agent_task.append([ele.action_cost[i],ele.service[i]])
            temp_agent_task.sort()

            for i, ele2 in enumerate(temp_agent_task):
                std_x = ele.pos[0] - 0.3 + i*0.3
                std_y = ele.pos[1] - 0.5
                task_shape = total_task.index(ele2[1]) % 5
                self.agent_service_order[name].append(task_shape)
                if task_shape == 0: #Square
                    self.agent_service[name][ele2[1]] = Rectangle((std_x-0.1, std_y-0.1), 0.2, 0.2, fc='royalblue', ec='black')
                elif task_shape == 1: #Star
                    sx = [std_x,std_x-0.025,std_x-0.1,std_x-0.025,std_x,std_x+0.025,std_x+0.1,std_x+0.025]
                    sy = [std_y+0.1,std_y+0.025,std_y,std_y-0.025,std_y-0.1,std_y-0.025,std_y,std_y+0.025]
                    self.agent_service[name][ele2[1]] = Polygon(xy=list(zip(sx,sy)), fc='yellow', ec='black')
                elif task_shape == 2: #Circle
                    self.agent_service[name][ele2[1]] = Circle((std_x,std_y), 0.1, fc='limegreen', ec='black')
                elif task_shape == 3: #Triangle
                    sx = [std_x, std_x-0.1, std_x+0.1]
                    sy = [std_y+0.13, std_y-0.07, std_y-0.07]
                    self.agent_service[name][ele2[1]] = Polygon(xy=list(zip(sx,sy)), fc='darkgray', ec='black')
                elif task_shape == 4: #Diamond
                    sx = [std_x, std_x-0.05, std_x, std_x+0.05]
                    sy = [std_y+0.1, std_y, std_y - 0.1, std_y]
                    self.agent_service[name][ele2[1]] = Polygon(xy=list(zip(sx,sy)), fc='brown', ec='black')
                self.patches.append(self.agent_service[name][ele2[1]])
                self.agent_service[name][ele2[1]].original_face_color = Task_Colors[task_shape]

        # Add Obj Ins
        for name, ele in self.obj_ins.items():
            for ins in ele:
                if name == 'waypoint':
                    if ins.instype == 'normal_way':
                        way_name = self.ax.text(ins.loc[0], ins.loc[1] + 0.20, ins.ind, fontsize = 8)
                        way_name.set_horizontalalignment('center')
                        way_name.set_verticalalignment('center')
                        self.artists.append(way_name)
                    else:
                        buffered_way = Point(ins.loc).buffer(0.05)
                        self.patches.append(PolygonPatch(buffered_way, fc = 'pink', ec='black', alpha=1.0, zorder=1.0))
                        way_name = self.ax.text(ins.loc[0], ins.loc[1] + 0.15, ins.ind, fontsize = 8)
                        way_name.set_horizontalalignment('center')
                        way_name.set_verticalalignment('center')
                        self.artists.append(way_name)
        # Add Time Text
        self.time_text = self.ax.text(maxx-0.1, y_bound-0.5, "Time: 0 sec")
        self.time_text.set_horizontalalignment('right')
        self.time_text.set_verticalalignment('center')
        self.time_text.set_fontweight('bold')
        self.artists.append(self.time_text)

        # title_name = "Map with " + str(self.env.num_of_goal) + " Goals and " + str(self.env.num_of_robot) + " Robots"
        title_name = "Every Action (EA)"
        self.title_text = self.ax.text((maxx+minx)/2, y_bound-0.5, title_name, fontsize = 'large')
        self.title_text.set_horizontalalignment('center')
        self.title_text.set_verticalalignment('center')
        self.title_text.set_fontweight('bold')
        self.artists.append(self.title_text)


        make_span = "MakeSpan: " + str(v_text[0])
        self.makespan_text = self.ax.text(maxx-0.1, y_bound-0.9, make_span)
        self.makespan_text.set_horizontalalignment('right')
        self.makespan_text.set_verticalalignment('center')
        self.makespan_text.set_fontweight('bold')
        self.artists.append(self.makespan_text)

        total = "Robot Total Cost: " + str(v_text[1])
        self.total_cost_text = self.ax.text(maxx-0.1, y_bound-1.3, total)
        self.total_cost_text.set_horizontalalignment('right')
        self.total_cost_text.set_verticalalignment('center')
        self.total_cost_text.set_fontweight('bold')
        self.artists.append(self.total_cost_text)

        # for name, ele in self.robots.items():
        #     robot_ind = ele.index
        #     robot_text = name + ":"
        #     for i in range(len(ele.service)):
        #         robot_text = robot_text + " " + ele.service[i] + " (" + str(ele.action_cost[i]) + ")"
        #     self.label_agent_names[name] = self.ax.text(maxx-0.1, y_bound - 1.7 - (0.4*robot_ind), robot_text, fontsize = 'small')
        #     self.label_agent_names[name].set_horizontalalignment('right')
        #     self.label_agent_names[name].set_verticalalignment('center')
        #     self.artists.append(self.label_agent_names[name])

        self.work_var = None

        # Add Goal Text
        for name, ele in self.goals_map.items():
            goal_center = ele['samples'][0]
            ext = list(ele['shape'].exterior.coords)
            upper_y = -100000
            for a in ext:
                upper_y = max(upper_y, a[1])
            # goal_index = ele['index']
            # a_name = name+':'
            # self.label_goal_names[name] = self.ax.text(minx+0.0, y_bound- 1.0-(0.4*goal_index), a_name)
            # self.label_goal_names[name].set_horizontalalignment('left')
            # self.label_goal_names[name].set_verticalalignment('center')
            # self.artists.append(self.label_goal_names[name])
            if ele['based'] == 'obstacle':
                self.goal_names[name] = self.ax.text(ele['shape'].centroid.coords[0][0], ele['shape'].centroid.coords[0][1], name, fontsize = 10)
            else:
                self.goal_names[name] = self.ax.text(goal_center[0], upper_y+0.2, name, fontsize = 10)
            self.goal_names[name].set_horizontalalignment('center')
            self.goal_names[name].set_verticalalignment('center')
            self.artists.append(self.goal_names[name])

            # self.label_goal_service[name] = dict()
            # for i, service_name in enumerate(ele['service']):
            #     ########## new world
            #     # self.label_goal_service[name][service_name] = self.ax.text(minx+1.6+(1.0*i), y_bound- 1.0 -(0.4*goal_index), service_name)
            #     # self.label_goal_service[name][service_name].set_horizontalalignment('left')
            #     # self.label_goal_service[name][service_name].set_verticalalignment('center')
            #     self.label_goal_service[name][service_name] = list()
            #     self.label_goal_service[name][service_name].append(self.ax.text(minx+1.2+(0.8*i), y_bound- 1.0 -(0.4*goal_index), service_name))
            #     # self.label_goal_service[name][service_name] = self.ax.text(minx+1.0+(0.8*i), y_bound- 1.0 -(0.4*goal_index), service_name)
            #     self.label_goal_service[name][service_name][0].set_horizontalalignment('left')
            #     self.label_goal_service[name][service_name][0].set_verticalalignment('center')
            #     self.label_goal_service[name][service_name].append(0)
            #     self.artists.append(self.label_goal_service[name][service_name][0])

        for name, ele in self.robot_path.items():
            cost_path = ele['path_cost']
            temp_path = []
            temp_time = 0.0
            path_len = len(cost_path)
            starts = None
            if path_len > 1:
                for i in range(1, path_len):
                    if i == 1:
                        if type(cost_path[i-1]) == list:
                            for partial in cost_path[i-1][2]:
                                if partial[1] == 2:
                                    temp_path.append((cost_path[i-1][0], temp_time))
                                    temp_time += round(partial[0], 2)
                                else:
                                    temp_path.append((cost_path[i-1][0], temp_time, partial[1], partial[2]))
                                    temp_time += round(partial[0], 2)
                            temp_path.append((cost_path[i-1][0], temp_time))
                            starts = cost_path[i-1][0]
                        else:
                            temp_path.append((cost_path[i-1], temp_time))
                            starts = cost_path[i-1]
                    if type(cost_path[i]) == list:
                        temp_time += round(get_distance(starts, cost_path[i][0]),2)
                        temp_path.append((cost_path[i][0], temp_time))
                        for partial in cost_path[i][2]:
                            if partial[1] == 2:
                                temp_time += round(partial[0], 2)
                                temp_path.append((cost_path[i][0], temp_time))
                            else:
                                temp_time += round(partial[0], 2)
                                temp_path.append((cost_path[i][0], temp_time, partial[1], partial[2]))
                        starts = cost_path[i][0]
                    else:
                        temp_time += round(get_distance(starts, cost_path[i]),2)
                        temp_path.append((cost_path[i], temp_time))
                        starts = cost_path[i]
                self.path_with_time[name] = temp_path
            else:
                self.path_with_time[name] = [(cost_path[0][0], temp_time)]

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 3) * 10,
                                                 interval=80,
                                                 blit=True)

    def save(self, file_name, speed):
        writervideo = animation.FFMpegWriter(fps = 10 * speed)
        self.animation.save(
            file_name,
            writer = writervideo
        )

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        # Init Goal position when t = 0
        if t ==0:
            pass
        set_time = str(round(t/10, 3))
        self.time_text.set_text("Time: " + set_time + " sec")

        # reset all colors
        for name, ele in self.agent_service.items():
            for name2, ele2 in ele.items():
                ele2.set_facecolor(ele2.original_face_color)

        for name, ele in self.path_with_time.items():
            pos = self.get_state_time(t / 10, ele, name)
            self.agents[name].center = (pos[0], pos[1])
            self.agent_names[name].set_position((pos[0], pos[1] + 0.4))
            for i, (name2, ele2) in enumerate(self.agent_service[name].items()):
                std_x = pos[0] - 0.5 + i*0.3
                std_y = pos[1] - 0.5
                task_shape = self.agent_service_order[name][i]
                if task_shape == 0: #Square
                     ele2.set_xy((std_x-0.1, std_y-0.1))
                elif task_shape == 1: #Star
                    sx = [std_x,std_x-0.025,std_x-0.1,std_x-0.025,std_x,std_x+0.025,std_x+0.1,std_x+0.025]
                    sy = [std_y+0.1,std_y+0.025,std_y,std_y-0.025,std_y-0.1,std_y-0.025,std_y,std_y+0.025]
                    ele2.set_xy(xy=list(zip(sx,sy)))
                elif task_shape == 2: #Circle
                    ele2.center = (std_x, std_y)
                elif task_shape == 3: #Triangle
                    sx = [std_x, std_x-0.1, std_x+0.1]
                    sy = [std_y+0.13, std_y-0.07, std_y-0.07]
                    ele2.set_xy(xy=list(zip(sx,sy)))
                elif task_shape == 4: #Diamond
                    sx = [std_x, std_x-0.05, std_x, std_x+0.05]
                    sy = [std_y+0.1, std_y, std_y - 0.1, std_y]
                    ele2.set_xy(xy=list(zip(sx,sy)))


        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                rr1 = d1.get_radius()
                rr2 = d2.get_radius()
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < rr1+rr2:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    #@staticmethod
    def get_state_time(self, t, path, r_name):
        if t <= path[0][1]:
            self.work_var = None
            return np.array(path[0][0])
        elif t >= path[-1][1]:
            self.work_var = None
            if len(path[-1]) == 4:
                self.task_patches[path[-1][3]][path[-1][2]].set_visible(False)
                # self.label_goal_service[path[-1][3]][path[-1][2]][1] = 2
            return np.array(path[-1][0])
        else:
            for i, time_ind in enumerate(path):
                if t < time_ind[1]:
                    i = i-1
                    break
            x1, y1 = path[i][0]
            x2, y2 = path[i+1][0]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)

            if len(path[i]) == 4:
                self.task_patches[path[i][3]][path[i][2]].set_visible(False)
                # self.label_goal_service[path[i][3]][path[i][2]][1] = 2
            if len(path[i+1]) == 4:
                self.task_patches[path[i+1][3]][path[i+1][2]].set_facecolor('red')
                self.agent_service[r_name][path[i+1][2]].set_facecolor('red')
                # self.label_goal_service[path[i+1][3]][path[i+1][2]][1] = 1
            if dx ==0 and dy == 0:
                if len(path[i+1]) ==4:
                    self.work_var = (path[i+1][2], path[i+1][3])
                else:
                    self.work_var = None
                return (x2, y2)
            pos_x = x1 + (t - path[i][1]) * 1 * math.cos(yaw)
            pos_y = y1 + (t - path[i][1]) * 1 * math.sin(yaw)
            pos = (pos_x, pos_y)
            self.work_var = None
            return pos

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos
