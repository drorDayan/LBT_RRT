from arr2_epec_seg_ex import *


# TODO make this class smarter (re-build for the whole tree for every node is bad)
# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points=None):
        if points is None:
            self.tree = Kd_tree([])
        else:
            self.tree = Kd_tree(points)

    def __tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = FT(0)
        # TODO: Consider with a custom distance
        search = K_neighbor_search(self.tree, query, k, epsilon, search_nearest, Euclidean_distance(), sort_neighbors)
        lst = []
        search.k_neighbors(lst)
        return lst

    def add_points(self, points):
        self.tree.insert(points)

    def get_nearest(self, point):
        nn = self.__tree_k_nn(point, 1)
        nn_in_tree = nn[0]
        return nn_in_tree[0]

    def get_k_nearest(self, point, k):
        nn = self.__tree_k_nn(point, k)
        return [nn_in_tree[0] for nn_in_tree in nn]
