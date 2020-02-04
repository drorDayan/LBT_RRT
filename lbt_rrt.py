from arr2_epec_seg_ex import *
import random
import time
import heapq
from collision_detector import CollisionDetectorFast, CollisionDetectorSlow
from neighbor_finder import NeighborsFinder
from math import sqrt, e, log

# Configurable Variables: #

k_nearest = 50
steer_eta = FT(0.6)

# Code: #


class LbtRrtNode:
    def __init__(self, v, parent=None, cost=FT(0)):
        # point_d
        self.v = v
        if parent is not None:
            self.g_inner_edges = {parent: cost}
            # the smallest full cost to get here and the parent which gives us that cost
            self.g_min_cost = (cost+parent.g_min_cost[0], parent)
            self.t_cost = parent.t_cost + cost
        else:
            self.g_inner_edges = {}
            # the smallest full cost to get here and the parent which gives us that cost
            self.g_min_cost = (FT(0), None)
            self.t_cost = FT(0)
        self.g_outer_edges = {}
        self.t_parent = parent

    def add_v_update_cost(self, cost_from_new_parent, new_parent, changed_nodes_prices):
        if cost_from_new_parent < self.g_min_cost[0]:
            changed_nodes_prices.append((cost_from_new_parent, self))
            self.g_min_cost = (cost_from_new_parent, new_parent)
            # update all childes
            sorted_childes = sorted(self.g_outer_edges.items(), key=lambda item: item[1])
            for (n, c) in sorted_childes:
                n.add_v_update_cost(cost_from_new_parent+c, self, changed_nodes_prices)
        return

    def remove_v_update_cost(self, v_from, changed_nodes_prices):
        if self.g_min_cost[1] != v_from:
            return
        else:
            changed_nodes_prices.append(self)
            min_item = min(self.g_inner_edges, key=lambda k: self.g_inner_edges[k]+k.g_min_cost[0])
            self.g_min_cost = (min_item.g_min_cost[0]+self.g_inner_edges[min_item], min_item)
            sorted_childes = sorted(self.g_outer_edges.items(), key=lambda item: item[1])
            for (n, c) in sorted_childes:
                n.remove_v_update_cost(self, changed_nodes_prices)
        return


class LbtRrtGraph:
    def __init__(self, robot_num, src):
        self.src = LbtRrtNode(src)
        self.num_of_nodes = 0
        self.robot_num = robot_num
        self.nodes = {src: self.src}

    def get_path_to_node(self, node, ret_path):
        curr_node = self.nodes[node]
        while curr_node is not None:
            ret_path.append(curr_node.v)
            curr_node = curr_node.t_parent
        ret_path.reverse()
        return

    def insert_new_node(self, p_from, p_new):
        if p_from == p_new:
            print("what just happen")
        curr_cost = path_cost(self.robot_num, p_from, p_new)
        from_node = self.nodes[p_from]
        new_node = LbtRrtNode(p_new, from_node, curr_cost)
        self.nodes[p_new] = new_node
        self.num_of_nodes += 1
        from_node.g_outer_edges[new_node] = curr_cost

    def insert_g_edge(self, v_from, v_to):
        if v_from == v_to:
            print("what just happen")
        curr_cost = path_cost(self.robot_num, v_from, v_to)
        to_node = self.nodes[v_to]
        from_node = self.nodes[v_from]
        from_node.g_outer_edges[to_node] = curr_cost
        to_node.g_inner_edges[from_node] = curr_cost
        changed_nodes_prices = []
        to_node.add_v_update_cost(curr_cost+from_node.g_min_cost[0], from_node, changed_nodes_prices)
        return changed_nodes_prices

    @staticmethod
    def delete_g_edge(v_from, v_to):
        if v_from == v_to:
            print("what just happen")
        del v_from.g_outer_edges[v_to]
        del v_to.g_inner_edges[v_from]
        changed_nodes_prices = []
        v_to.remove_v_update_cost(v_from, changed_nodes_prices)
        return changed_nodes_prices


# class RrtNode:
#     def __init__(self, pt, pr=None, n=0):
#         self.point = pt
#         self.parent = pr
#         self.nval = n  # Will be used to store distance to this point, etc.
#
#     def get_path_to_here(self, ret_path):
#         cur = self
#         while cur is not None:
#             ret_path.insert(0, cur.point)
#             cur = cur.parent
#         return ret_path


def path_cost(robot_num, p1, p2):
    return distance_squared(robot_num, p1, p2)


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


def try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector, robot_num):
    nn = neighbor_finder.get_k_nearest(dest_point, k_nearest)
    valid_neighbors = []
    for neighbor in nn:
        free = collision_detector.path_collision_free(neighbor, dest_point)
        if free:
            valid_neighbors.append(neighbor)
    if len(valid_neighbors) > 0:
        best = min(valid_neighbors, key=lambda n: graph.nodes[n].t_cost + path_cost(robot_num, n, dest_point))
        graph.insert_new_node(best, dest_point)
        return True
    return False


# return e*(1+1/d) as in the OMPL implementation who claims that the RRT* used constant
def get_k_rrg(robot_num):
    return e + (e/(2*robot_num))


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


def consider_edge(graph, x1, x2, epsilon, collision_detector):
    if x1 == x2:
        return
    changed_nodes = graph.insert_g_edge(x1, x2)
    heapq.heapify(changed_nodes)
    while len(changed_nodes) > 0:
        x = changed_nodes[0]
        if x[1].t_cost > (FT(1)+epsilon)*x[0]:
            g_parent = x[1].g_min_cost[1]
            if collision_detector.path_collision_free(g_parent.v, x[1].v):
                x[1].t_parent = g_parent
                x[1].t_cost = g_parent.t_cost+g_parent.g_outer_edges[x[1]]
                heapq.heappop(changed_nodes)
            else:
                # is invalid edge, remove it and update costs
                graph.delete_g_edge(g_parent, x[1])

                # TODO currently ignoring the result of delete_g_edge (which is the changed vertices)
                # update changed_nodes and their costs
                changed_nodes = [(node.g_min_cost[0], node) for _, node in changed_nodes]
                heapq.heapify(changed_nodes)
        else:
            heapq.heappop(changed_nodes)
    return


def generate_path(path, robots, obstacles, destination, epsilon=FT(1/30), time_to_run=60):
    # random.seed(0)  # for tests
    start = time.time()
    print("running, epsilon = ", epsilon, "time to run = ", time_to_run)
    robot_num = len(robots)
    robot_width = FT(1)
    validate_input(robots, destination, robot_width)
    k_rrg = get_k_rrg(robot_num)
    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    collision_detector = CollisionDetectorFast(robot_width, obstacles, robot_num)

    vertices = [start_point]
    graph = LbtRrtGraph(robot_num, start_point)
    neighbor_finder = NeighborsFinder(vertices)
    while time.time()-start < time_to_run:
        new_point = Point_d(2*robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2*robot_num)])
        near = neighbor_finder.get_nearest(new_point)
        new = steer(robot_num, near, new_point, steer_eta)
        free = collision_detector.path_collision_free(near, new)
        if free:
            vertices.append(new)
            graph.insert_new_node(near, new)
            neighbor_finder.add_points([new])

            # rewiring phase
            curr_nn_to_consider = neighbor_finder.get_k_nearest(new, 1+int(log(graph.num_of_nodes) * k_rrg))
            for neighbor in curr_nn_to_consider:
                consider_edge(graph, neighbor, new, epsilon, collision_detector)
            for neighbor in curr_nn_to_consider:
                consider_edge(graph, new, neighbor, epsilon, collision_detector)

    if try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector, robot_num):
        d_path = []
        graph.get_path_to_node(dest_point, d_path)
        for dp in d_path:
            path.append([Point_2(dp[2 * i], dp[2 * i + 1]) for i in range(robot_num)])
        print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices),
              "cost = ", graph.nodes[dest_point].t_cost)
    else:
        print("no path found in time")
        print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices))
