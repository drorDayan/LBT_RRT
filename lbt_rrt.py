from arr2_epec_seg_ex import *
import random
import time
from math import sqrt

# Configurable Variables: #

k_nearest = 50
steer_eta = FT(0.6)
FREESPACE = 'freespace'

# Code: #


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


# obs collision detection code:
# noinspection PyArgumentList
class CollisionDetector:
    def __init__(self, robot_width, obstacles, robot_num):
        # init obs for collision detection
        one_width_square = Polygon_2(self.get_origin_robot_coord(robot_width))
        double_width_square = Polygon_with_holes_2(Polygon_2(self.get_origin_robot_coord(FT(2) * robot_width)))
        inflated_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
        c_space_obstacles = [minkowski_sum_by_full_convolution_2(one_width_square, obs) for obs in inflated_obstacles]
        c_space_arrangements = [self.polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
        self.obstacles_arrangement = self.overlay_multiple_arrangements(c_space_arrangements,
                                                                        self.merge_faces_by_freespace_flag)
        self.obstacles_point_locator = Arr_trapezoid_ric_point_location(self.obstacles_arrangement)
        self.double_width_square_arrangement = self.polygon_with_holes_to_arrangement(double_width_square)
        self.double_width_square_point_locator = Arr_trapezoid_ric_point_location(self.double_width_square_arrangement)
        self.robot_num = robot_num

    @staticmethod
    def polygon_with_holes_to_arrangement(poly):
        assert isinstance(poly, Polygon_with_holes_2)
        arr = Arrangement_2()
        insert(arr, [Curve_2(e) for e in poly.outer_boundary().edges()])

        # set the freespace flag for the only current two faces
        for f in arr.faces():
            assert isinstance(f, Face)
            f.set_data({FREESPACE: f.is_unbounded()})

        for hole in poly.holes():
            insert(arr, [Curve_2(e) for e in hole.edges()])

        for f in arr.faces():
            assert isinstance(f, Face)
            if f.data() is None:
                f.set_data({FREESPACE: True})
        return arr

    @staticmethod
    def get_origin_robot_coord(width):
        robot_width = width / FT(2)
        v1 = Point_2(robot_width, robot_width)
        v2 = Point_2(robot_width * FT(-1), robot_width)
        v3 = Point_2(robot_width * FT(-1), robot_width * FT(-1))
        v4 = Point_2(robot_width, robot_width * FT(-1))
        return [v1, v2, v3, v4]

    @staticmethod
    def merge_faces_by_freespace_flag(x, y):
        return {FREESPACE: x[FREESPACE] and y[FREESPACE]}

    @staticmethod
    def overlay_multiple_arrangements(arrs, face_merge_func):
        final_arr = arrs[0]
        for arr in arrs[1:]:
            temp_res = Arrangement_2()

            overlay(final_arr, arr, temp_res, Arr_face_overlay_traits(face_merge_func))
            final_arr = temp_res
        return final_arr

    @staticmethod
    def is_in_free_face(point_locator, point):
        face = Face()
        # locate can return a vertex or an edge or a face
        located_obj = point_locator.locate(point)
        # if located_obj.is_vertex():
        #     return False
        # if located_obj.is_halfedge():
        #     return False
        if located_obj.is_face():
            located_obj.get_face(face)
            return face.data()[FREESPACE]
        return False

    @staticmethod
    def get_normal_movement_vector(p1, p2, i, j):
        start_point = [p1[2*i] - p1[2*j], p1[2*i+1] - p1[2*j+1]]
        diff_vec = [(p2[2*i] - p1[2*i])-(p2[2*j] - p1[2*j]), (p2[2*i+1] - p1[2*i+1])-(p2[2*j+1] - p1[2*j+1])]
        normal_movement_vector = Curve_2(Point_2(start_point[0], start_point[1]),
                                         Point_2(start_point[0]+diff_vec[0], start_point[1]+diff_vec[1]))
        return normal_movement_vector

    def two_robot_intersect(self, p1, p2, i, j):
        mov_vec = self.get_normal_movement_vector(p1, p2, i, j)
        return do_intersect(self.double_width_square_arrangement, mov_vec)

    # checks for collisions return:
    # True if collision free
    # False, first robot index that touches an obs if a robot touches an obs
    # False, robot_num if a robot touches another robot
    def path_collision_free(self, p1, p2):
        # check for obs collision
        for i in [i for i in range(self.robot_num)]:
            if do_intersect(self.obstacles_arrangement, Curve_2(Point_2(p1[2*i], p1[2*i+1]),
                                                                Point_2(p2[2*i], p2[2*i+1]))):
                return False
        # check for robot to robot collision
        for i in range(self.robot_num):
            for j in range(i + 1, self.robot_num):
                if self.two_robot_intersect(p1, p2, i, j):
                    return False
        return True


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


# def get_batch(robot_num, num_of_points, min_coord, max_coord):
#     # num_of_points_in_dest_direction = random.randint(0, num_of_points/5)
#     # v1 = [Point_d(2*robot_num,
#     #              [(FT(random.uniform(min_coord, max_coord))+dest_p[i])/FT(2) for i in range(2*robot_num)])
#     #      for j in range(num_of_points_in_dest_direction)]
#     batch = [Point_d(2*robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2*robot_num)])
#              for _ in range(num_of_points)]
#     return batch


def get_min_max(obstacles):
    max_x = max(max(v.x() for v in obs) for obs in obstacles)
    max_y = max(max(v.y() for v in obs) for obs in obstacles)
    min_x = min(min(v.x() for v in obs) for obs in obstacles)
    min_y = min(min(v.y() for v in obs) for obs in obstacles)
    assert min_x == min_y and max_x == max_y, "scene should be square"
    min_coord = min(min_x, min_y)
    max_coord = min(max_x, max_y)
    return min_coord.to_double(), max_coord.to_double()


def get_square_mid(robot):
    x = (robot[0].x()+robot[1].x())/FT(2)
    y = (robot[1].y()+robot[2].y())/FT(2)
    return [x, y]


def distance_squared(robot_num, p1, p2):
    return sum([(p1[i] - p2[i]) * (p1[i] - p2[i]) for i in range(2*robot_num)], FT(0))


# noinspection PyArgumentList
def steer(robot_num, near, rand, eta):
    dist = FT(sqrt(distance_squared(robot_num, near, rand).to_double()))
    if dist < eta:
        return rand
    else:
        return Point_d(2*robot_num, [near[i]+(rand[i]-near[i])*eta/dist for i in range(2*robot_num)])


def try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
    nn = neighbor_finder.get_k_nearest(dest_point, k_nearest)
    for neighbor in nn:
        free = collision_detector.path_collision_free(neighbor, dest_point)
        if free:
            graph[dest_point] = RrtNode(dest_point, graph[neighbor])
            return True
    return False


def get_start_and_dest(robots, destination):
    robot_num = len(robots)
    start_ref_points = [get_square_mid(robot) for robot in robots]
    target_ref_points = [[dest.x(), dest.y()] for dest in destination]
    start_point = Point_d(2*robot_num, sum(start_ref_points, []))
    dest_point = Point_d(2*robot_num, sum(target_ref_points, []))
    return start_point, dest_point


def validate_input(robots, destination, robot_width):
    robot_num = len(robots)
    assert len(destination) == robot_num, "robot amount and destination amount mismatch"
    for i in range(robot_num):
        curr_robot_width = robots[i][1].x() - robots[i][0].x()
        assert curr_robot_width == robot_width, "robot width is assumed to be 1"


def generate_path(path, robots, obstacles, destination):
    # random.seed(0)  # for tests
    start = time.time()
    robot_num = len(robots)
    robot_width = FT(1)
    validate_input(robots, destination, robot_width)

    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    collision_detector = CollisionDetector(robot_width, obstacles, robot_num)

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
        if i % 1000 == 100:
            # print("vertices amount: ", len(vertices), "time= ", time.time() - start)
            if try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
                break
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    # print("k_nearest = ", k_nearest)
    # print("steer_eta = ", steer_eta)
    # print("num_of_points_in_batch = ", num_of_points_in_batch)
    # print("used single robot movement:", do_use_single_robot_movement)
    print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices), "steer_eta = ", steer_eta)
