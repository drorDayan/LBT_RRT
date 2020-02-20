import lbt_rrt
import srm_rrt
from arr2_epec_seg_ex import *
import time
import matplotlib.pyplot as plt
import gc


img_folder = "dror3_run_imgs"+"/"


def test_cost_per_time(robots, obstacles, destination):
    num_of_tries = 3
    time_to_run = 240
    print("running test_cost_per_time")
    path = []
    for fast_cd in [False, True]:
        eps_s = [0.001, 0.01, 0.1, 0.25, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 20]
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
        fast_cd_str = "fast" if fast_cd else "slow"

        plt.plot(eps_s, [r[0] for r in res], color="red")
        plt.title("vertices as a function of epsilon using" + fast_cd_str + "collision detection")
        plt.xlabel("epsilon")
        plt.savefig(img_folder+str(time_to_run) + "_vertices for epsilon" + fast_cd_str + "collision detection.png")
        plt.close()

        plt.plot(eps_s, [r[1] for r in res], color="red")
        plt.title("cost as a function of epsilon using" + fast_cd_str + "collision detection")
        plt.xlabel("epsilon")
        plt.savefig(img_folder+str(time_to_run) + "_cost for epsilon" + fast_cd_str + "collision detection.png")
        plt.close()

        plt.plot(eps_s, [r[1]/r[0] for r in res], color="red")
        plt.title("cost per vertex as a function of epsilon using" + fast_cd_str + "collision detection")
        plt.xlabel("epsilon")
        plt.savefig(img_folder+str(time_to_run) + "_cost per vertex" + fast_cd_str + "collision detection.png")
        plt.close()
    print("finish test_cost_per_time")


def test_cost_over_time(robots, obstacles, destination):
    num_of_tries = 5
    time_to_run = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]
    print("running test_cost_over_time")
    path = []
    for fast_cd in [False, True]:
        eps_s = [0.001, 0.01, 0.1, 0.25, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 20]
        res = [[[0, 0] for _ in range(len(time_to_run))] for _ in range(len(eps_s))]
        eps_index = 0
        for eps in eps_s:
            for i in range(num_of_tries):
                start_time = time.time()
                c_res = lbt_rrt.generate_path(path, robots, obstacles, destination, time_to_run, FT(eps), fast_cd)
                v, cost = c_res[len(c_res) - 1]
                print("time:", time.time()-start_time, "vertices:", v, "cost:", cost, "epsilon:", eps,
                      "fast_collision_detection:", fast_cd)
                for time_to_run_index in range(len(time_to_run)):
                    res[eps_index][time_to_run_index][0] += v
                    res[eps_index][time_to_run_index][1] += cost.to_double()
                path = []
                gc.collect()
            for time_to_run_index in range(len(time_to_run)):
                res[eps_index][time_to_run_index][0] /= num_of_tries
                res[eps_index][time_to_run_index][1] /= num_of_tries
            eps_index += 1
        # Print data and graphs
        print(res)
        fast_cd_str = "fast" if fast_cd else "slow"

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
        plt.title("cost as a function of time using" + fast_cd_str + "collision detection")
        plt.savefig(img_folder + "cost as a function of time," + fast_cd_str + "collision detection.png")
        plt.close()
    print("finish test_cost_over_time")


# TODO test for success percentage as function of time (and epsilon)
# TODO better scene
# TODO assume we are allowed a fix number of vertices
# this function is called for testing lbt_rrt with multiple parameters
def generate_path(path, robots, obstacles, destination):
    print("running tests")
    # test_cost_per_time(robots, obstacles, destination)
    test_cost_over_time(robots, obstacles, destination)
    print("finish")
