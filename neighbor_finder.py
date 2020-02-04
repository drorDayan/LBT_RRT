from arr2_epec_seg_ex import *


# TODO make this class smarter (re-build for the whole tree for every node is bad)
# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points=None):
        self.big_tree_points = []
        self.big_tree = Kd_tree([])
        if points is None:
            self.small_tree = Kd_tree([])
            self.small_tree_points = []
        else:
            self.small_tree = Kd_tree(points)
            self.small_tree_points = points

    def __tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = FT(0)
        # TODO: Consider with a custom distance
        search1 = K_neighbor_search(self.small_tree, query, k, epsilon, search_nearest, Euclidean_distance(),
                                    sort_neighbors)
        search2 = K_neighbor_search(self.big_tree, query, k, epsilon, search_nearest, Euclidean_distance(),
                                    sort_neighbors)
        lst1 = []
        search1.k_neighbors(lst1)
        lst2 = []
        search2.k_neighbors(lst2)
        return lst1, lst2

    def add_points(self, points):
        if len(self.small_tree_points) < 300:
            self.small_tree.insert(points)
            self.small_tree_points += points
        else:
            self.big_tree.insert(self.small_tree_points + points)
            self.big_tree_points += self.small_tree_points + points
            self.small_tree_points = []
            self.small_tree = Kd_tree([])

    def get_nearest(self, point):
        nn1, nn2 = self.__tree_k_nn(point, 1)
        if len(nn2) == 0:
            return nn1[0][0]
        nn2_in_tree = nn2[0]
        if len(nn1) == 0:
            return nn2_in_tree[0]
        nn1_in_tree = nn1[0]
        if nn1_in_tree[1] < nn2_in_tree[1]:
            return nn1_in_tree[0]
        return nn2_in_tree[0]

    def get_k_nearest(self, point, k):
        nn1, nn2 = self.__tree_k_nn(point, k)
        if len(nn2) == 0:
            return [nn_in_tree[0] for nn_in_tree in nn1]
        if len(nn1) == 0:
            return [nn_in_tree[0] for nn_in_tree in nn1]

        size_1 = len(nn1)
        size_2 = len(nn2)
        res = [0]*min(k, size_1+size_2)
        i, j = 0, 0
        while i < size_1 and j < size_2 and i+j < k:
            if nn1[i][1] < nn2[j][1]:
                res[i+j] = nn1[i][0]
                i += 1
            else:
                res[i+j] = nn2[j][0]
                j += 1
        while i < size_1 and i+j < k:
            res[i+j] = nn1[i][0]
            i += 1
        while j < size_2 and i+j < k:
            res[i+j] = nn2[j][0]
            j += 1
        return res
