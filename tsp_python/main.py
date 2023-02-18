
import sys
# from time import time

# from brkga_mp_ipr.enums import Sense
# from brkga_mp_ipr.types_io import load_configuration
# from brkga_mp_ipr.algorithm import BrkgaMpIpr

from tsp_instance import TSPInstance
# from tsp_decoder import TSPDecoder
# import greedy_tour as gt

from anneal import SA
# from grasp import GRASP
import common


# def brkga(instance_file):
#     # if len(sys.argv) < 4:
#     #     print("Usage: python main_minimal.py <seed> <config-file> "
#     #           "<num-generations> <tsp-instance-file>")
#     #     sys.exit(1)

#     # seed = int(sys.argv[1])
#     # config_file = sys.argv[2]
#     # num_generations = int(sys.argv[3])
#     # instance_file = sys.argv[4]

#     seed            = 1
#     config_file     = "config.conf"
#     num_generations = 50
#     # instance_file   = "instances/burma14.dat"    

#     print(f"BRKGA reading data from {instance_file}")
#     instance = TSPInstance(instance_file)

#     print(f"BRKGA  reading parameters from {config_file}")
#     brkga_params, _ = load_configuration(config_file)

#     print("Building BRKGA data and initializing...")

#     decoder = TSPDecoder(instance)

#     brkga = BrkgaMpIpr(
#         decoder=decoder,
#         sense=Sense.MINIMIZE,
#         seed=seed,
#         chromosome_size=instance.num_nodes,
#         params=brkga_params
#     )

#     brkga.initialize()


#     print(f"BRKGA evolving {num_generations} generations...")
#     brkga.evolve(num_generations)

#     best_cost = brkga.get_best_fitness()
#     print(f"BRKGA best cost: {best_cost}")

#     return best_cost

def greedy(instance_file):
    
    print(f"\nGreedy reading data from {instance_file}")

    instance = common.Instance(instance_file)

    print(f"Running the next neighbor heuristic.")

    _, cost = instance.initialTour()

    print(f"Greedy best cost: {cost}")

    return cost


def dpTSP(instance_file):

    print(f"DynProg reading data from {instance_file}")
    instance = TSPInstance(instance_file)

    n = instance.num_nodes

    print(f"Running the dynamic program for TSP.")

    # memoization for top down recursion
    memo = [[-1]*(1 << (n+1)) for _ in range(n+1)]

    def fun(i, mask):
        # base case
        # if only ith bit and 1st bit is set in our mask,
        # it implies we have visited all other nodes already
        if mask == ((1 << i) | 3):
            return instance.distance(1, i)

        # memoization
        if memo[i][mask] != -1:
            return memo[i][mask]

        res = 10**9 # result of this sub-problem

        # we have to travel all nodes j in mask and end the path at ith node
        # so for every node j in mask, recursively calculate cost of
        # travelling all nodes in mask
        # except i and then travel back from node j to node i taking
        # the shortest path take the minimum of all possible j nodes
        for j in range(1, n+1):
            if (mask & (1 << j)) != 0 and j != i and j != 1:
                res = min( res, fun(j, mask & (~(1 << i))) + instance.distance(j, i) )
        memo[i][mask] = res # storing the minimum value
        return res

    # # Driver program to test above logic
    res = 10**9
    for i in range(1, n+1):
        # try to go from node 1 visiting all nodes in between to i
        # then return from i taking the shortest route to 1
        res = min(res, fun(i, (1 << (n+1))-1) + instance.distance(i,1))

    print(f"DynProg best cost: {res}")

    return res


if __name__ == "__main__":

    # instance_file   = "instances/burma14.dat"
    # instance_file   = "instances/brazil58.dat"
    # instance_file   = "instances/rd400.dat" 
    # instance_file   = "instances/vm1084.dat" 

    # start = time()
    # greedy_val = greedy(instance_file)
    # elapsed = time() - start
    # elapsed *= 1000
    # print(f"Greedy runtime {elapsed:.1f}ms\n")  


    # cd /home/celio/Projects/iRace_tunning/ && $IRACE_HOME/bin/irace

    # python -m main instances/burma14.dat 6 1000
    # python -m main instance_file         T stopping_iter
    # python -m main instances/brazil58.dat 5 1000

    #python main.py instances/burma14.dat 5 500

    # print(sys.argv[0])

    seed          = int(sys.argv[1])
    instance_file = sys.argv[2]
    T             = float(sys.argv[3])
    stopping_iter = int(sys.argv[4])

    stopping_T = 1e-8
    alpha      = 1. - T / float(stopping_iter)

    sa = SA(instance_file, T, alpha, stopping_T, stopping_iter, seed)
    sa.anneal()

    # start = time()
    # sa = SA(instance_file, stopping_iter=5000)
    # sa.anneal()
    # elapsed = time() - start
    # elapsed *= 1000
    # print(f"SA runtime {elapsed:.1f}ms")  
    # improvement = 100 * (greedy_val - sa.best_cost) / (greedy_val)
    # print(f"SA improvement over greedy heuristic: {improvement : .2f}%\n")  

    # start = time()
    # gr = GRASP(instance_file, 2)
    # gr.grasp()
    # elapsed = time() - start
    # elapsed *= 1000
    # print(f"GRASP runtime {elapsed:.1f}ms")
    # if gr.best_cost == None:
    #     gr.best_cost = 0 
    # improvement = 100 * (greedy_val - gr.best_cost) / (greedy_val)
    # print(f"GRASP improvement over greedy heuristic: {improvement : .2f}%\n")  



    # if instance_file  == "instances/burma14.dat":
    #     start = time()
    #     opt_cost = dpTSP(instance_file)
    #     elapsed = time() - start
    #     elapsed *= 1000
    #     print(f"DynProg runtime {elapsed:.1f}ms") 
    #     improvement = 100 * (greedy_val - opt_cost) / (greedy_val)
    #     print(f"DynProg improvement over greedy heuristic: {improvement : .2f}%\n")

    # start = time()
    # best_cost = brkga(instance_file)
    # elapsed = time() - start
    # elapsed *= 1000
    # print(f"BRKGA runtime {elapsed:.1f}ms")
    # improvement = 100 * (greedy_val - best_cost) / (greedy_val)
    # print(f"BRKGA improvement over greedy heuristic: {improvement : .2f}%\n")

    # if instance_file  == "instances/burma14.dat":   
    #     improvement = 100 * (opt_cost - best_cost) / (opt_cost)
    #     print(f"BRKGA distance from DynProg solution: {improvement : .2f}%\n")       
