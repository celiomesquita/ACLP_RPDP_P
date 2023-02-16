import copy
import random
from tsp_instance import TSPInstance

class Instance(object):
    def __init__(self, instance_file):
        self.TSP = TSPInstance(instance_file)
        self.N = self.TSP.num_nodes
        self.nodes = [i for i in range(self.N)]
        self.best_cost = 0.0
        self.best_tour = []
        self.cost_list = []

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

        tourCost = self.getCost(tour)
        if tourCost < self.best_cost:  # If best found so far, update best cost
            self.best_cost = tourCost
            self.best_tour = tour
        self.cost_list.append(tourCost)

        tour     = self.twoOpt(tour)

        tourCost = self.getCost(tour)

        return tour, tourCost

    def getDist(self, node_0, node_1):
        return self.TSP.distance(node_0, node_1)

    def getCost(self, tour):
        N = len(tour)
        cost = 0
        for i, _ in enumerate(tour):
            cost += self.getDist(tour[i % N], tour[(i + 1) % N])
        return cost

    def twoOpt(self, initialTour):

        tour         = copy.deepcopy(initialTour)
        testing_tour = copy.deepcopy(tour)

        for i in range(0, len(tour) - 2):

            for j in range(i+1, len(tour) - 1):

                segment_cost = self.getCost(testing_tour[i:j+1])

                testing_tour[i:j+1] = list(reversed(testing_tour[i:j+1]))  

                new_cost = self.getCost(testing_tour[i:j+1])

                if (new_cost < segment_cost):

                    for n in range(0, len(tour)): 

                        tour[n] = testing_tour[n]          
        return tour