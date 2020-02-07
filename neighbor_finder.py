from arr2_epec_seg_ex import *
from rrt_common import distance_squared
import heapq


# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, robot_num, points=None):
        self.robot_num = robot_num
        if points is None:
            self.array = []
        else:
            self.array = points
        # trees array will have max size, points_array, kd_tree
        self.trees = [[130, [], Kd_tree([])]]

    def __tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = FT(0)
        res = []
        for _, points_array, kd_tree in self.trees:
            if len(points_array) > 0:
                search = K_neighbor_search(kd_tree, query, k, epsilon, search_nearest, Euclidean_distance(),
                                           sort_neighbors)
                lst = []
                search.k_neighbors(lst)
                res += lst
        return res

    def add_points(self, points):
        if len(self.array) < 64:
            self.array += points
        else:
            num_of_points = len(self.array) + len(points)
            i = 0
            while i < len(self.trees):
                if self.trees[i][0] > num_of_points+len(self.trees[i][1]):
                    points_to_add = self.array + points
                    self.array = []
                    for j in range(0, i):
                        points_to_add += self.trees[j][1]
                        self.trees[j][1] = []
                        self.trees[j][2] = Kd_tree([])
                    self.trees[i][1] += points_to_add
                    self.trees[i][2].insert(points_to_add)
                    return
                i += 1
            points_to_add = self.array + points
            self.array = []
            for i in range(0, len(self.trees)):
                points_to_add += self.trees[i][1]
                self.trees[i][1] = []
                self.trees[i][2] = Kd_tree([])
            self.trees.append([self.trees[len(self.trees) - 1][0]*2, points_to_add, Kd_tree(points_to_add)])
            print(self.trees[len(self.trees) - 1][0])
            return

    def get_nearest(self, point):
        nn = self.__tree_k_nn(point, 1)
        if len(nn) == 0:
            return min(self.array, key=lambda p: distance_squared(self.robot_num, point, p))
        nn_min = min(nn, key=lambda n: n[1])
        if len(self.array) == 0:
            return nn_min[0]
        arr_min = min(self.array, key=lambda p: distance_squared(self.robot_num, point, p))
        if nn_min[1] < distance_squared(self.robot_num, point, arr_min):
            return nn_min[0]
        else:
            return arr_min

    def get_k_nearest(self, point, k):
        nn = self.__tree_k_nn(point, k)
        heap_nn = [(d, n) for n, d in nn] + [(distance_squared(self.robot_num, point, n), n) for n in self.array]
        heapq.heapify(heap_nn)
        return [heapq.heappop(heap_nn)[1] for _ in range(k)]


