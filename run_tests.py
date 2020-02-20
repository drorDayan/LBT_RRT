import lbt_rrt
import srm_rrt
from arr2_epec_seg_ex import *
import time
import matplotlib.pyplot as plt
import gc


img_folder = "tmptmptmp/"


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
        print(res)
        plt.plot(eps_s, [r[0] for r in res], color="red")
        if fast_cd:
            plt.title("vertices as a function of epsilon using fast collision detection")
            plt.savefig(img_folder+str(time_to_run) + "_vertices for epsilon fast collision detection.png")
        else:
            plt.title("vertices as a function of epsilon using slow collision detection")
            plt.savefig(img_folder+str(time_to_run) + "_vertices for epsilon slow collision detection.png")
        plt.close()
        plt.plot(eps_s, [r[1] for r in res], color="red")
        if fast_cd:
            plt.title("cost as a function of epsilon using fast collision detection")
            plt.savefig(img_folder+str(time_to_run) + "_cost for epsilon fast collision detection.png")
        else:
            plt.title("cost as a function of epsilon using slow collision detection")
            plt.savefig(img_folder+str(time_to_run) + "_cost for epsilon slow collision detection.png")
        plt.close()

        plt.plot(eps_s, [r[1]/r[0] for r in res], color="red")
        if fast_cd:
            plt.title("cost per vertex as a function of epsilon using fast collision detection")
            plt.savefig(img_folder+str(time_to_run) + "_cost per vertex for epsilon fast collision detection.png")
        else:
            plt.title("cost per vertex as a function of epsilon using slow collision detection")
            plt.savefig(img_folder+str(time_to_run) + "_cost per vertex for epsilon slow collision detection.png")
        plt.close()

    print("finish test_cost_per_time")


# TODO test for success percentage as function of time (and epsilon)
# TODO solution quality (cost) as a function of time (per epsilon)
# TODO better scene
# TODO assume we are allowed a fix number of vertices
# this function is called for testing lbt_rrt with multiple parameters
def generate_path(path, robots, obstacles, destination):
    print("running tests")
    test_cost_per_time(robots, obstacles, destination)
    print("finish")
