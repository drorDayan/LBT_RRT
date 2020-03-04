import lbt_rrt
import srm_rrt
from arr2_epec_seg_ex import *
import time
import matplotlib.pyplot as plt
import gc


img_folder = "dror11_run_imgs"+"/run6/"
# eps_s = [0.05, 0.1, 0.25, 0.5, 0.75, 1, 2]
# eps_s = [0.001, 0.01, 0.1, 1, 10, 1000000]
# eps_s = [pow(10, x) for x in range(-5, 6)]
eps_s = [0.25*(x+1) for x in range(10)]


def test_cost_per_time(robots, obstacles, destination, time_to_run):
    num_of_tries = 10
    print("running test_cost_per_time")
    path = []
    for fast_cd in [False, True]:
        # eps_s = [0.001, 0.01, 0.1, 0.25, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 20]
        res = [[0, 0] for _ in range(len(eps_s))]
        eps_index = 0
        for eps in eps_s:
            for i in range(num_of_tries):
                start_time = time.time()
                c_res = lbt_rrt.generate_path(path, robots, obstacles, destination, [time_to_run], FT(eps), fast_cd)
                v, cost = c_res[0]
                print("time:", time.time()-start_time, "vertices:", v, "cost:", cost, "epsilon:", eps,
                      "fast_collision_detection:", fast_cd)
                if cost == lbt_rrt.no_path_found_cost:
                    print("invalid data, did not find path")
                    print(fast_cd, eps, i)
                res[eps_index][0] += v
                res[eps_index][1] += cost.to_double()
                path = []
                gc.collect()
            res[eps_index][0] /= num_of_tries
            res[eps_index][1] /= num_of_tries
            eps_index += 1
        # Print data and graphs
        print(res)
        fast_cd_str = " fast " if fast_cd else " slow "

        plt.xscale("log")
        plt.plot(eps_s, [r[0] for r in res], color="red")
        plt.title("vertices as a function of epsilon using" + fast_cd_str + "collision detection")
        plt.xlabel("epsilon")
        plt.savefig(img_folder+str(time_to_run) + "_vertices for epsilon" + fast_cd_str + "collision detection.png")
        plt.close()

        plt.xscale("log")
        plt.plot(eps_s, [r[1] for r in res], color="red")
        plt.title("cost as a function of epsilon using" + fast_cd_str + "collision detection")
        plt.xlabel("epsilon")
        plt.savefig(img_folder+str(time_to_run) + "_cost for epsilon" + fast_cd_str + "collision detection.png")
        plt.close()

        plt.xscale("log")
        plt.plot(eps_s, [r[1]/r[0] for r in res], color="red")
        plt.title("cost per vertex as a function of epsilon using" + fast_cd_str + "collision detection")
        plt.xlabel("epsilon")
        plt.savefig(img_folder+str(time_to_run) + "_cost per vertex" + fast_cd_str + "collision detection.png")
        plt.close()
    print("finish test_cost_per_time")


def test_cost_over_time(robots, obstacles, destination):
    num_of_tries = 10
    time_to_run = [(i+1)*5 for i in range(18)]
    print("running test_cost_over_time")
    path = []
    for fast_cd in [False, True]:
        res = [[[0, 0, 0, 0] for _ in range(len(time_to_run))] for _ in range(len(eps_s))]
        eps_index = 0
        for eps in eps_s:
            for i in range(num_of_tries):
                start_time = time.time()
                c_res = lbt_rrt.generate_path(path, robots, obstacles, destination, time_to_run, FT(eps), fast_cd)
                v, cost = c_res[len(c_res) - 1]
                print("time:", time.time()-start_time, "vertices:", v, "cost:", cost, "epsilon:", eps,
                      "fast_collision_detection:", fast_cd)
                for time_to_run_index in range(len(time_to_run)):
                    res[eps_index][time_to_run_index][0] += c_res[time_to_run_index][0]
                    res[eps_index][time_to_run_index][1] += c_res[time_to_run_index][1].to_double()
                    if c_res[time_to_run_index][1] != lbt_rrt.no_path_found_cost:
                        res[eps_index][time_to_run_index][2] += 1
                        res[eps_index][time_to_run_index][3] += c_res[time_to_run_index][1].to_double()

                path = []
                gc.collect()
            for time_to_run_index in range(len(time_to_run)):
                res[eps_index][time_to_run_index][0] /= num_of_tries
                res[eps_index][time_to_run_index][1] /= num_of_tries
                if res[eps_index][time_to_run_index][2] > 0:
                    res[eps_index][time_to_run_index][3] /= res[eps_index][time_to_run_index][2]
            eps_index += 1
        # Print data and graphs
        print(res)
        fast_cd_str = " fast " if fast_cd else " slow "

        for eps_index in range(len(eps_s)):
            plt.plot(time_to_run, [r[0] for r in res[eps_index]], label=str(eps_s[eps_index]))
        plt.legend()
        plt.xlabel("time (seconds)")
        plt.title("vertices as a function of time using" + fast_cd_str + "collision detection")
        plt.savefig(img_folder + "vertices as a function of time," + fast_cd_str + "collision detection.png")
        plt.close()

        for eps_index in range(len(eps_s)):
            plt.plot(time_to_run, [r[1] for r in res[eps_index]], label=str(eps_s[eps_index]))
        plt.legend()
        plt.xlabel("time (seconds)")
        plt.title("cost (with penalty) as a function of time using" + fast_cd_str + "collision detection")
        plt.savefig(img_folder + "cost with penalty as a function of time," + fast_cd_str + "collision detection.png")
        plt.close()

        for eps_index in range(len(eps_s)):
            plt.plot(time_to_run, [r[3] for r in res[eps_index]], label=str(eps_s[eps_index]))
        plt.legend()
        plt.xlabel("time (seconds)")
        plt.title("cost (without penalty) as a function of time using" + fast_cd_str + "collision detection")
        plt.savefig(img_folder + "cost without penalty as a function of time," + fast_cd_str + "collision detection.png")
        plt.close()

        for eps_index in range(len(eps_s)):
            plt.plot(time_to_run, [r[2] for r in res[eps_index]], label=str(eps_s[eps_index]))
        plt.legend()
        plt.xlabel("time (seconds)")
        plt.title("success rate as a function of time using" + fast_cd_str + "collision detection")
        plt.savefig(img_folder + "success rate as a function of time," + fast_cd_str + "collision detection.png")
        plt.close()

    print("finish test_cost_over_time")


def test_cost_per_vertices(robots, obstacles, destination, num_of_vertices):
    num_of_tries = 10
    print("running test_cost_per_vertices", num_of_vertices)
    path = []
    for fast_cd in [True]:
        res = [[0, 0, 0, 0] for _ in range(len(eps_s))]
        eps_index = 0
        for eps in eps_s:
            for i in range(num_of_tries):
                start_time = time.time()
                c_res = lbt_rrt.generate_path(path, robots, obstacles, destination, [9999999], FT(eps), fast_cd, num_of_vertices)
                v, cost = c_res[len(c_res) - 1]
                print("time:", time.time()-start_time, "vertices:", v, "cost:", cost, "epsilon:", eps,
                      "fast_collision_detection:", fast_cd)
                res[eps_index][0] += c_res[0][0]
                res[eps_index][1] += c_res[0][1].to_double()
                if c_res[0][1] != lbt_rrt.no_path_found_cost:
                    res[eps_index][2] += 1
                    res[eps_index][3] += c_res[0][1].to_double()

                path = []
                gc.collect()
            res[eps_index][0] /= num_of_tries
            res[eps_index][1] /= num_of_tries
            if res[eps_index][2] > 0:
                res[eps_index][3] /= res[eps_index][2]
            eps_index += 1
        # Print data and graphs
        print(res)
        fast_cd_str = " fast " if fast_cd else " slow "

        plt.xscale("log")
        plt.plot(eps_s, [r[1] for r in res], color="red")
        plt.title("cost (with penalty) as a function of epsilon for "+str(num_of_vertices)+" vertices")# + fast_cd_str + "c-d")
        plt.xlabel("epsilon")
        plt.savefig(
            img_folder + fast_cd_str + str(num_of_vertices) + "_cost (with penalty) as a function of epsilon for a fixed number of vertices.png")
        plt.close()

        plt.xscale("log")
        plt.plot(eps_s, [r[3] for r in res], color="red")
        plt.title("cost (without penalty) as a function of epsilon for "+str(num_of_vertices)+" vertices")# + fast_cd_str + "c-d")
        plt.xlabel("epsilon")
        plt.savefig(
            img_folder + fast_cd_str + str(num_of_vertices) + "_cost (without penalty) as a function of epsilon for a fixed number of vertices.png")
        plt.close()

    print("finish test_cost_per_vertices")


# def test_success_over_time(robots, obstacles, destination):
#     num_of_tries = 2
#     time_to_run = [(i+1)*10 for i in range(20)]  # , 105, 120, 135, 150, 165, 180, 195, 210, 225, 240]  # , 270, 300, 330, 360]
#     print("running test_success_over_time")
#     path = []
#     for fast_cd in [False, True]:
#         # eps_s = [0.001, 0.01, 0.1, 0.25, 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15]
#         # eps_s = [0.001, 0.01, 0.1, 0.5, 1, 2.5, 5, 7.5, 10, 15]
#         res = [[0 for _ in range(len(time_to_run))] for _ in range(len(eps_s))]
#         eps_index = 0
#         for eps in eps_s:
#             for i in range(num_of_tries):
#                 start_time = time.time()
#                 c_res = lbt_rrt.generate_path(path, robots, obstacles, destination, time_to_run, FT(eps), fast_cd)
#                 v, cost = c_res[len(c_res) - 1]
#                 print("time:", time.time()-start_time, "vertices:", v, "cost:", cost, "epsilon:", eps,
#                       "fast_collision_detection:", fast_cd)
#                 for time_to_run_index in range(len(time_to_run)):
#                     if c_res[time_to_run_index][1] != lbt_rrt.no_path_found_cost:
#                         res[eps_index][time_to_run_index] += 1
#                 path = []
#                 gc.collect()
#             eps_index += 1
#         # Print data and graphs
#         print(res)
#         fast_cd_str = " fast " if fast_cd else " slow "
#
#         for eps_index in range(len(eps_s)):
#             plt.plot(time_to_run, [r for r in res[eps_index]], label=str(eps_s[eps_index]))
#         plt.legend()
#         plt.xlabel("time (seconds)")
#         plt.title("success rate as a function of time using" + fast_cd_str + "collision detection")
#         plt.savefig(img_folder + "success rate as a function of time," + fast_cd_str + "collision detection.png")
#         plt.close()
#     print("finish test_success_over_time")


# TODO better scene?
# this function is called for testing lbt_rrt with multiple parameters
def generate_path(path, robots, obstacles, destination):
    print("running tests")
    # test_cost_over_time(robots, obstacles, destination)
    # test_cost_per_vertices(robots, obstacles, destination, 1000)
    # test_cost_per_vertices(robots, obstacles, destination, 3000)
    # test_cost_per_vertices(robots, obstacles, destination, 5000)
    test_cost_per_vertices(robots, obstacles, destination, 10000)
    # test_cost_per_vertices(robots, obstacles, destination, 15000)
    # test_cost_per_time(robots, obstacles, destination, 60)
    test_cost_per_time(robots, obstacles, destination, 120)
    # test_cost_per_time(robots, obstacles, destination, 180)
    test_cost_per_time(robots, obstacles, destination, 240)
    test_cost_per_time(robots, obstacles, destination, 480)
    print("finish")
