import math
import random
import numpy as np
import common
import copy

RNG = np.random.default_rng()

# RCL the Restricted Candidates List
class RCL(object):
    def __init__(self, size, nbhood):
        self.size = size
        self.list = []
        if len(nbhood) >= size:
            i = 0
            for _ in range(size):
                # greedilly assemble the RCL
                node = nbhood[i]
                i += 1
                # nbhood.pop(0)                
                self.list.append(node)

    def randomPick(self):
        i = math.floor(RNG.random()*self.size)
        e = self.list[i]
        self.list.pop(i)
        return e

class GRASP(object):
    def __init__(self, instance_file, stopping_iter=1000, rcl_size=1):

        print(f"GRASP reading data from {instance_file}")

        self.instance = common.Instance(instance_file)

        self.N = self.instance.TSP.num_nodes

        self.stopping_iter = stopping_iter
        self.iteration = 1

        self.nodes = [i for i in range(self.N)]

        self.best_tour = None
        self.best_cost = float("Inf")
        self.cost_list = []
        self.rcl_size = rcl_size

    def moveToSolution(self, rcl, not_visited, solution):
        next_node = rcl.randomPick()
        not_visited.remove(next_node)
        solution.append(next_node)

    def grasp(self):

        # Initialize with the greedy tour.
        initialTour, initialCost = self.instance.initialTour()

        self.best_cost = initialCost
        self.best_tour = copy.deepcopy(initialTour)

        print(f"Running GRASP for {self.stopping_iter} iterations | RCL size = {self.rcl_size}")


        while self.iteration < self.stopping_iter:

            not_visited = copy.deepcopy(self.best_tour) # select the not visited nodes

            rcl = RCL(self.rcl_size, not_visited)

            # always pick a node from the RCl and put in the solution
            solution = []

            while not_visited:
                
                # shrink the RCL
                if len(not_visited) < self.rcl_size :
                    self.rcl_size = len(not_visited)

                # complete the RCL
                if len(rcl.list) < self.rcl_size:
                    rcl.list.append(not_visited[0])

                rcl = RCL(self.rcl_size, not_visited)

                # always pick a node from the RCl and put in the solution
                self.moveToSolution(rcl, not_visited, solution)

            cost = self.instance.getCost(solution)

            if cost < self.best_cost:  # If best found so far, update best cost
                self.best_cost = cost
                self.best_tour = solution

            self.iteration += 1

        self.best_tour = self.instance.twoOpt(self.best_tour)
        self.best_cost = self.instance.getCost(self.best_tour)

        print(f"GRASP best cost: {self.best_cost}")
