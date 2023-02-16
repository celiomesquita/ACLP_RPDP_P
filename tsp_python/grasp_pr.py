import math
import random
import numpy as np
from tsp_instance import TSPInstance
import greedy_tour as gt

RNG = np.random.default_rng()

def pickFirstNode(vector):
    node = vector[0]
    vector.pop(0)
    return node

def pickRandomNode(vector):
    n = len(vector)
    i = math.floor(RNG.random()*n)
    node = vector[i]
    vector.pop(i)
    return node

# RCL the Restricted Candidates List
class RCL(object):
    def __init__(self, size, nbhood): # neighborhood
        self.size = size
        self.list = []
        if len(nbhood) >= size:
            for _ in range(size):
                # greedilly assemble the RCL
                e = pickFirstNode(nbhood)
                self.list.append(e)

class GRASP(object):
    def __init__(self, instance_file, T=-1, alpha=-1, stopping_T=-1, stopping_iter=-1):

        print(f"GRASP reading data from {instance_file}")

        self.instance = TSPInstance(instance_file)

        self.N = self.instance.num_nodes
        self.stopping_iter = 1000 if stopping_iter == -1 else stopping_iter
        self.iteration = 1

        self.nodes = [i for i in range(self.N)]

        self.best_tour = None
        self.best_cost = float("Inf")
        self.tour_cost_list = []
        self.rcl_size = math.floor(math.sqrt( len(self.N) ))

    def isFeasible(node, free_nodes):

        if node in free_nodes:
            return True
        return False

    def initialTour(self):
        """
        Greedy algorithm to get an initial tour (closest-neighbour).
        """
        cur_node = random.choice(self.nodes)  # start from a random node
        tour = [cur_node]

        free_nodes = set(self.nodes)
        free_nodes.remove(cur_node)
        while free_nodes:
            next_node = min(free_nodes, key=lambda x: self.getDist(cur_node, x))  # nearest neighbour
            free_nodes.remove(next_node)
            tour.append(next_node)
            cur_node = next_node

        tour_cost = self.getTourCost(tour)
        if tour_cost < self.best_cost:  # If best found so far, update best cost
            self.best_cost = tour_cost
            self.best_tour = tour
        self.tour_cost_list.append(tour_cost)
        return tour, tour_cost


    def getDist(self, node_0, node_1):
        return self.instance.distance(node_0, node_1)

    def getTourCost(self, tour):
        """
        Total distance of the current tour path.
        """
        tour_cost = 0
        for i in range(self.N):
            tour_cost += self.getDist(tour[i % self.N], tour[(i + 1) % self.N])
        return tour_cost


    def grasp(self):
        # Initialize with the greedy tour.
        # self.cur_tour, self.cur_cost = self.initialTour()

        print("Executing GRASP (no Path Relinking yet)...")

        Ntemp  = [] # a temporary list of nodes
        Nprime = []

        while self.iteration < self.stopping_iter:
    
            free_nodes = set(self.nodes) 

            rcl = RCL(self.rcl_size, free_nodes)

            Nlocal = []

            while free_nodes:

                if len(rcl.list) < self.rcl_size:
                    break

                next_node = pickRandomNode(rcl.list) # pick from the RCL a candidate node

                for node in rcl.list:
                    Ntemp.append(node)

                if self.isFeasible(next_node, free_nodes) : # Check all constraints
                    Nlocal.append(next_node)
                    free_nodes.remove(next_node)

                rcl = RCL(self.rcl_size, Nprime) 


                self.tour_cost_list.append(self.cur_cost)

            self.iteration += 1

        print(f"GRASP best cost: {self.best_cost}")
