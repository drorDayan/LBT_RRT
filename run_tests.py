import lbt_rrt
import srm_rrt
from arr2_epec_seg_ex import *
import time
import matplotlib.pyplot as plt

num_of_tries = 3
time_to_run = 150


# TODO test for success percentage as function of time (and epsilon)
# TODO solution quality (cost) as a function of time (per epsilon)
# this function is called for testing lbt_rrt with multiple parameters
def generate_path(path, robots, obstacles, destination):
    print("running tests")
    # for i in range(50):
    #     srm_rrt.generate_path(path, robots, obstacles, destination)
    for fast_cd in [False, True]:
        eps_s = [0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1, 2.5, 5, 10, 15]
        res = [[0, 0] for _ in range(len(eps_s))]
        eps_index = 0
        for eps in eps_s:
            for i in range(num_of_tries):
                start_time = time.time()
                v, cost = lbt_rrt.generate_path(path, robots, obstacles, destination, FT(eps), time_to_run, fast_cd)
                print("time:", time.time()-start_time, "vertices:", v, "cost:", cost, "epsilon:", eps,
                      "fast_collision_detection:", fast_cd)
                if cost == FT(99999):
                    print("invalid data, did not find path")
                    print("for i=", i)
                res[eps_index][0] += v
                res[eps_index][1] += cost.to_double()
                path = []
            res[eps_index][0] /= num_of_tries
            res[eps_index][1] /= num_of_tries
            eps_index += 1
        print(res)
        plt.plot(eps_s, [r[0] for r in res], color="red")
        if fast_cd:
            plt.title("vertices as a function of epsilon using fast collision detection")
            plt.savefig(str(time_to_run) + "_vertices for epsilon fast collision detection.png")
        else:
            plt.title("vertices as a function of epsilon using slow collision detection")
            plt.savefig(str(time_to_run) + "_vertices for epsilon slow collision detection.png")
        plt.close()
        plt.plot(eps_s, [r[1] for r in res], color="red")
        if fast_cd:
            plt.title("cost as a function of epsilon using fast collision detection")
            plt.savefig(str(time_to_run) + "_cost for epsilon fast collision detection.png")
        else:
            plt.title("cost as a function of epsilon using slow collision detection")
            plt.savefig(str(time_to_run) + "_cost for epsilon slow collision detection.png")
        plt.close()

        plt.plot(eps_s, [r[1]/r[0] for r in res], color="red")
        if fast_cd:
            plt.title("cost per vertex as a function of epsilon using fast collision detection")
            plt.savefig(str(time_to_run) + "_cost per vertex for epsilon fast collision detection.png")
        else:
            plt.title("cost per vertex as a function of epsilon using slow collision detection")
            plt.savefig(str(time_to_run) + "_cost per vertex for epsilon slow collision detection.png")
        plt.close()

    print("finish")
