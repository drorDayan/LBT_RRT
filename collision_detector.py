from arr2_epec_seg_ex import *
from math import sqrt

FREESPACE = 'freespace'


# obs collision detection code:
# noinspection PyArgumentList
class CollisionDetectorFast:
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
        insert(arr, [Curve_2(edge) for edge in poly.outer_boundary().edges()])

        # set the freespace flag for the only current two faces
        for f in arr.faces():
            assert isinstance(f, Face)
            f.set_data({FREESPACE: f.is_unbounded()})

        for hole in poly.holes():
            insert(arr, [Curve_2(edge) for edge in hole.edges()])

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

    def __two_robot_intersect(self, p1, p2, i, j):
        mov_vec = self.get_normal_movement_vector(p1, p2, i, j)
        return do_intersect(self.double_width_square_arrangement, mov_vec)

    # checks for collisions return:
    # True if collision free
    # False, if not
    def path_collision_free(self, p1, p2):
        # check for obs collision
        for i in [i for i in range(self.robot_num)]:
            if do_intersect(self.obstacles_arrangement, Curve_2(Point_2(p1[2*i], p1[2*i+1]),
                                                                Point_2(p2[2*i], p2[2*i+1]))):
                return False
        # check for robot to robot collision
        for i in range(self.robot_num):
            for j in range(i + 1, self.robot_num):
                if self.__two_robot_intersect(p1, p2, i, j):
                    return False
        return True


inflation_epsilon = FT(0.01)


# noinspection PyArgumentList
class CollisionDetectorSlow:
    def __init__(self, robot_width, obstacles, robot_num):
        self.robot_num = robot_num
        inf_sq_coord = (robot_width+inflation_epsilon)/FT(2)
        v1 = Point_2(inf_sq_coord, inf_sq_coord)
        v2 = Point_2(inf_sq_coord * FT(-1), inf_sq_coord)
        v3 = Point_2(inf_sq_coord * FT(-1), inf_sq_coord*FT(-1))
        v4 = Point_2(inf_sq_coord, inf_sq_coord * FT(-1))
        inflated_square = Polygon_2([v1, v2, v3, v4])
        cgal_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
        c_space_obstacles = [minkowski_sum_by_full_convolution_2(inflated_square, obs) for obs in cgal_obstacles]
        arrangements = [self.polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
        single_arrangement = self.overlay_multiple_arrangements(arrangements, self.merge_faces_by_freespace_flag)
        self.point_locator = Arr_landmarks_point_location(single_arrangement)

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

    def is_valid_config(self, point_locator, conf, robot_num, epsilon):
        for j in range(robot_num):
            if not self.is_in_free_face(point_locator, Point_2(conf[2 * j], conf[2 * j + 1])):
                return False
            for k in range(j + 1, robot_num):
                if abs(FT.to_double(conf[2 * j] - conf[2 * k])) < 1 + epsilon.to_double() and \
                        abs(FT.to_double(conf[2 * j + 1] - conf[2 * k + 1])) < 1 + epsilon.to_double():
                    return False
        return True

    # checks for collisions return:
    # True if collision free
    # False, if not
    def path_collision_free(self, p1, p2):
        max_robot_path_len = FT(0)
        for i in range(self.robot_num):
            robot_path_len = (p2[2 * i] - p1[2 * i]) * (p2[2 * i] - p1[2 * i]) + \
                             (p2[2 * i + 1] - p1[2 * i + 1]) * (p2[2 * i + 1] - p1[2 * i + 1])
            if robot_path_len > max_robot_path_len:
                max_robot_path_len = robot_path_len
        sample_amount = FT(sqrt(max_robot_path_len.to_double())) / inflation_epsilon + FT(1)
        diff_vec = [((p2[i] - p1[i]) / sample_amount) for i in range(2 * self.robot_num)]
        curr = [p1[i] for i in range(2 * self.robot_num)]
        for i in range(int(sample_amount.to_double())):
            curr = [sum(x, FT(0)) for x in zip(curr, diff_vec)]
            if not self.is_valid_config(self.point_locator, curr, self.robot_num, inflation_epsilon):
                return False
        return True
