import math
import random
from tsp_instance import TSPInstance
import greedy_tour as gt
import numpy as np
import common

RNG = np.random.default_rng()

# RCL the Restricted Candidates List
class RCL(object):
    def __init__(self, size, nbhood):
        self.size = size
        self.list = []
        if len(nbhood) >= size:
            for _ in range(size):
                # greedilly assemble the RCL
                node = nbhood[0]
                nbhood.pop(0)                
                self.list.append(node)

    def randomPick(self):
        i = math.floor(RNG.random()*self.size)
        e = self.list[i]
        self.list.pop(i)
        return e

class GRASP(object):
    def __init__(self, instance_file, stopping_T=-1, stopping_iter=-1):

        print(f"GRASP reading data from {instance_file}")

        self.instance = TSPInstance(instance_file)

        self.N = self.instance.num_nodes
        self.stopping_iter = 1000 if stopping_iter == -1 else stopping_iter
        self.iteration = 1

        self.nodes = [i for i in range(self.N)]

        self.best_tour = None
        self.best_cost = float("Inf")
        self.cost_list = []


    def grasp(self):

        # Initialize with the greedy tour.
        initialTour, initialCost = common.initialTour(self.instance)

        self.best_cost = initialCost

        print("Running GRASP...")

        # rcl_size = math.floor( self.N / 4 )

        rcl_size = 3

        Ntemp  = []

        while self.iteration < self.stopping_iter:

            rcl = RCL(rcl_size, initialTour) # edges are drawed from not visited nodes

            cur_node = random.choice(initialTour)  # start from a random node

            not_visited = set(initialTour) # select the not visited nodes
            not_visited.remove(cur_node)
            growing_tour = [cur_node]

            while not_visited:
                
                if len(rcl.list) == 0:
                    break

                next_node = rcl.randomPick()
                growing_tour.append(next_node)

                for node in rcl.list: # save in a temporary growing_tour the nodes remaining in the RCL.
                    Ntemp.append(node)

                rcl = RCL(rcl_size, initialTour) # mount a new RCL
                
                if len(rcl.list) < rcl_size:
                    rcl = RCL(rcl_size, Ntemp)

            cost = common.getCost(self.instance, growing_tour)

            if cost < self.best_cost:  # If best found so far, update best cost
                self.best_cost = cost
                self.best_tour = growing_tour

        self.best_tour = common.twoOpt(self.instance, self.best_tour)

        self.best_cost = common.getCost(self.instance, self.best_tour)

        print(f"GRASP best cost: {self.best_cost}")
