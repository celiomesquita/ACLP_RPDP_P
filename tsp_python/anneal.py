import math
import random
import common

class SA(object):
    def __init__(self, instance_file, T=-1, alpha=-1, stopping_T=-1, stopping_iter=-1):

        print(f"SA reading data from {instance_file}")

        self.instance = common.Instance(instance_file)

        self.N = self.instance.TSP.num_nodes

        self.T = math.sqrt(self.N) if T == -1 else T
        self.T_save = self.T  # save inital T to reset if batch annealing is used
        self.alpha = 0.995 if alpha == -1 else alpha
        self.stopping_temperature = 1e-8 if stopping_T == -1 else stopping_T
        self.stopping_iter = 1000 if stopping_iter == -1 else stopping_iter
        self.iteration = 1

        self.nodes = [i for i in range(self.N)]

        self.best_solution = None
        self.best_cost = float("Inf")
        self.cost_list = []


    def p_accept(self, candidate_cost):
        """
        Probability of accepting if the candidate is worse than current.
        Depends on the current temperature and difference between candidate and current.
        """
        return math.exp(-abs(candidate_cost - self.tourCost) / self.T)

    def accept(self, candidate):
        """
        Accept with probability 1            if candidate is better than current.
        Accept with probability p_accept(..) if candidate is worse.
        """
        candidate_cost = self.instance.getCost(candidate)
        if candidate_cost < self.tourCost:
            self.tourCost, self.initialTour = candidate_cost, candidate
            if candidate_cost < self.best_cost:
                self.best_cost, self.best_solution = candidate_cost, candidate
        else:
            if random.random() < self.p_accept(candidate_cost):
                self.tourCost, self.initialTour = candidate_cost, candidate

    def anneal(self):
        """
        Execute simulated annealing algorithm.
        """
        # Initialize with the greedy solution.
        self.initialTour, self.tourCost = self.instance.initialTour()

        print("Running annealing...")
        while self.T >= self.stopping_temperature and self.iteration < self.stopping_iter:
            candidate = list(self.initialTour)
            l = random.randint(2, self.N - 1)
            i = random.randint(0, self.N - l)
            candidate[i : (i + l)] = reversed(candidate[i : (i + l)])
            self.accept(candidate)
            self.T *= self.alpha
            self.iteration += 1

            self.cost_list.append(self.tourCost)

        self.best_tour = self.instance.twoOpt(self.initialTour)

        self.best_cost = self.instance.getCost(self.best_tour)

        print(f"SA best cost: {self.best_cost}")
