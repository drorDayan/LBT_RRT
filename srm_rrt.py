import random
import time
from collision_detector import CollisionDetectorFast, CollisionDetectorSlow, DenseSpaceQuery
from neighbor_finder import NeighborsFinder
from rrt_common import *

# Configurable Variables: #

k_nearest = 50
steer_eta = FT(0.8)


class RrtNode:
    def __init__(self, pt, pr=None, n=0):
        self.point = pt
        self.parent = pr
        self.nval = n  # Will be used to store distance to this point, etc.

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path


def try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
    nn = neighbor_finder.get_k_nearest(dest_point, k_nearest)
    for neighbor in nn:
        free = collision_detector.path_collision_free(neighbor, dest_point)
        if free:
            graph[dest_point] = RrtNode(dest_point, graph[neighbor])
            return True
    return False


def generate_path(path, robots, obstacles, destination):
    # random.seed(0)  # for tests
    start = time.time()
    robot_num = len(robots)
    robot_width = FT(1)
    validate_input(robots, destination, robot_width)
    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    dense_space_query = DenseSpaceQuery(robot_width, obstacles, robot_num)
    collision_detector = CollisionDetectorFast(robot_width, obstacles, robot_num)

    vertices = [start_point]
    graph = {start_point: RrtNode(start_point)}
    neighbor_finder = NeighborsFinder(vertices)
    i = 0
    while True:
        i += 1
        new_point = Point_d(2*robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2*robot_num)])
        near = neighbor_finder.get_nearest(new_point)
        new = steer(robot_num, near, new_point, steer_eta)
        free = collision_detector.path_collision_free(near, new)
        if free:
            vertices.append(new)
            graph[new] = RrtNode(new, graph[near])
            neighbor_finder.add_points([new])
        else:
            rids = dense_space_query.robots_is_in_dense(near) + dense_space_query.robots_is_in_dense(new)
            for rid in rids:
                new_data = [near[j] for j in range(2 * robot_num)]
                new_data[2 * rid] = new[2 * rid]
                new_data[2 * rid + 1] = new[2 * rid + 1]
                my_new = Point_d(2 * robot_num, new_data)
                free = collision_detector.path_collision_free(near, my_new)
                if free:
                    vertices.append(my_new)
                    graph[my_new] = RrtNode(my_new, graph[near])
                    neighbor_finder.add_points([my_new])

        if i % 100 == 0:
            if try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
                break
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices), "steer_eta = ", steer_eta)


# for i in range(robot_num):
#     new_data = [near[j] for j in range(2 * robot_num)]
#     new_data[2 * i] = new[2 * i]
#     new_data[2 * i + 1] = new[2 * i + 1]
#     my_new = Point_d(2 * robot_num, new_data)
#     free, aa = path_collision_free(obstacles_point_locator, robot_num, near, my_new, obstacles_arrangement,
#                                    double_width_square_arrangement, double_width_square_point_locator, do_single=True,
#                                    robot_idx=i, first_invalid_idx=idx)
#     if free:
#         new_points.append(my_new)
#         vertices.append(my_new)
#         graph[my_new] = RRT_Node(my_new, graph[near])
