import lbt_rrt
from arr2_epec_seg_ex import *
import time

num_of_tries = 3


# this function is called for testing lbt_rrt with multiple parameters
def generate_path(path, robots, obstacles, destination):
    for fast_cd in [True, False]:
        eps_s = [FT(0.001), FT(0.01), FT(0.1), FT(1), FT(2), FT(10)]
        res = [[0, 0]]*len(eps_s)
        eps_index = 0
        for eps in eps_s:
            for i in range(num_of_tries):
                start_time = time.time()
                vertices, cost = lbt_rrt.generate_path(path, robots, obstacles, destination, eps, 60, fast_cd)
                print("time:", time.time()-start_time, "vertices:", vertices, "cost:", cost, "epsilon:", eps,
                      "fast_collision_detection:", fast_cd)
                res[eps_index][0] += vertices
                res[eps_index][1] += cost
            eps_index += 1
            res[eps_index][0] /= num_of_tries
            res[eps_index][1] /= num_of_tries
        print("here we need to plot")
        print(res)
    print("finish")
