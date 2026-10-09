import math
import numpy as np
import scipy.spatial

class KDTree:
    """
 Nearest neighbor search class with KDTree
 """

    def __init__(self, data):
        # make Tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
 search NN

 inp: input data, single frame or multi frame
 """
        if type(inp) == tuple:
            dist, index = self.tree.query(inp, k=k)
            # k == 1 (small maps make knn collapse to 1) returns scalars;
            # normalize so callers can always iterate.
            return np.atleast_1d(index), np.atleast_1d(dist)
        elif type(inp) == list:
            dist = []
            index = []
            for i in inp:
                idist, iindex = self.tree.query(i, k=k)
                dist.append(idist)
                index.append(iindex)
            return index, dist
        else:
            print("Type error")

        return NULL

    def search_in_distance(self, inp, r):
        """
 find points with in a distance r
 """
        index = self.tree.query_ball_point(inp, r)
        return index

class DisjointSet:
    def __init__(self, n):
        self.data = [-1 for _ in range(n)]
        self.size = n

    def upward(self, change_list, index):
        value = self.data[index]
        if value < 0:
            return index

        change_list.append(index)
        return self.upward(change_list, value)

    def find(self, index):
        change_list = []
        result = self.upward(change_list, index)

        for i in change_list:
            self.data[i] = result
        return result

    def union(self, x, y):
        x = self.find(x)
        y = self.find(y)

        if x == y:
            return
        if self.data[x] < self.data[y]:
            self.data[y] = x
        elif self.data[x] > self.data[y]:
            self.data[x] = y
        else:
            self.data[x] -= 1
            self.data[y] = x

        self.size -= 1

def compare_nodes(n1, n2):
    return n1['cost'] < n2['cost']

def get_distance(a1,a2):
    sx, sy = a1
    gx, gy = a2
    dx = gx-sx
    dy = gy-sy
    return math.sqrt(dx**2 + dy**2)

def get_sum_of_cost(path):
    rst = 0
    start = path[0]
    for i in range(1,len(path)):
        rst += get_distance(start,path[i])
        start = path[i]
    return rst

def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def length(v):
  return math.sqrt(dotproduct(v, v))

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def cal_midway(waypoint_list):
    n = len(waypoint_list)
    if n < 2:
        return [waypoint_list[0][0], waypoint_list[0][1]]
    #count = 2
    for i in range(1, n):
        way1_one = waypoint_list[0][0]
        way1_two = waypoint_list[0][1]
        way2_one = waypoint_list[i][0]
        way2_two = waypoint_list[i][1]
        if get_distance(way1_one, way2_one) + get_distance(way1_two, way2_two) >\
            get_distance(way1_one, way2_two) + get_distance(way1_two, way2_one):
            start = (round((way1_one[0]*i + way2_two[0])/(i+1),3), round((way1_one[1]*i + way2_two[1])/(i+1),3))
            end = (round((way1_two[0]*i + way2_one[0])/(i+1),3), round((way1_two[1]*i + way2_one[1])/(i+1),3))
            waypoint_list[0] = [start, end]
        else:
            start = (round((way1_one[0]*i + way2_one[0])/(i+1),3), round((way1_one[1]*i + way2_one[1])/(i+1),3))
            end = (round((way1_two[0]*i + way2_two[0])/(i+1),3), round((way1_two[1]*i + way2_two[1])/(i+1),3))
            waypoint_list[0] = [start, end]

    #print(waypoint_list[0])
    return waypoint_list[0]
