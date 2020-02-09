from arr2_epec_seg_ex import *
from math import sqrt


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


# noinspection PyArgumentList
def distance_squared(robot_num, p1, p2):
    return Euclidean_distance().transformed_distance(p1, p2)


# noinspection PyArgumentList
def steer(robot_num, near, rand, eta):
    dist = FT(sqrt(distance_squared(robot_num, near, rand).to_double()))
    if dist < eta*eta:
        return rand
    else:
        return Point_d(2*robot_num,
                       [near[i]+(rand[i]-near[i])*FT(sqrt((eta/dist).to_double())) for i in range(2*robot_num)])


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
    assert robot_num > 0, "robot amount must be greater then 0"
    for i in range(robot_num):
        curr_robot_width = robots[i][1].x() - robots[i][0].x()
        assert curr_robot_width == robot_width, "robot width is assumed to be 1"
