import math
import random
from tsp_instance import TSPInstance
import greedy_tour as gt

class SA(object):
    def __init__(self, instance_file, T=-1, alpha=-1, stopping_T=-1, stopping_iter=-1):

        print(f"SA reading data from {instance_file}")

        self.instance = TSPInstance(instance_file)

        _, self.tour = gt.greedy_tour(self.instance)

        self.N = len(self.tour)
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

    def initial_solution(self):
        """
        Greedy algorithm to get an initial solution (closest-neighbour).
        """
        cur_node = random.choice(self.nodes)  # start from a random node
        solution = [cur_node]

        free_nodes = set(self.nodes)
        free_nodes.remove(cur_node)
        while free_nodes:
            next_node = min(free_nodes, key=lambda x: self.dist(cur_node, x))  # nearest neighbour
            free_nodes.remove(next_node)
            solution.append(next_node)
            cur_node = next_node

        cur_fit = self.cost(solution)
        if cur_fit < self.best_cost:  # If best found so far, update best cost
            self.best_cost = cur_fit
            self.best_solution = solution
        self.cost_list.append(cur_fit)
        return solution, cur_fit


    def dist(self, node_0, node_1):
        return self.instance.distance(node_0, node_1)

    def cost(self, solution):
        """
        Total distance of the current solution path.
        """
        cur_fit = 0
        for i in range(self.N):
            cur_fit += self.dist(solution[i % self.N], solution[(i + 1) % self.N])
        return cur_fit

    def p_accept(self, candidate_cost):
        """
        Probability of accepting if the candidate is worse than current.
        Depends on the current temperature and difference between candidate and current.
        """
        return math.exp(-abs(candidate_cost - self.cur_cost) / self.T)

    def accept(self, candidate):
        """
        Accept with probability 1            if candidate is better than current.
        Accept with probability p_accept(..) if candidate is worse.
        """
        candidate_cost = self.cost(candidate)
        if candidate_cost < self.cur_cost:
            self.cur_cost, self.cur_solution = candidate_cost, candidate
            if candidate_cost < self.best_cost:
                self.best_cost, self.best_solution = candidate_cost, candidate
        else:
            if random.random() < self.p_accept(candidate_cost):
                self.cur_cost, self.cur_solution = candidate_cost, candidate

    def anneal(self):
        """
        Execute simulated annealing algorithm.
        """
        # Initialize with the greedy solution.
        self.cur_solution, self.cur_cost = self.initial_solution()


        print("Starting annealing...")
        while self.T >= self.stopping_temperature and self.iteration < self.stopping_iter:
            candidate = list(self.cur_solution)
            l = random.randint(2, self.N - 1)
            i = random.randint(0, self.N - l)
            candidate[i : (i + l)] = reversed(candidate[i : (i + l)])
            self.accept(candidate)
            self.T *= self.alpha
            self.iteration += 1

            self.cost_list.append(self.cur_cost)

        print(f"SA best cost: {self.best_cost}")

        # improvement = 100 * (self.cost_list[0] - self.best_cost) / (self.cost_list[0])
        # print(f"Improvement over greedy heuristic: {improvement : .2f}%")

    def batch_anneal(self, times=10):
        """
        Execute simulated annealing algorithm `times` times, with random initial solutions.
        """
        for i in range(1, times + 1):
            print(f"Iteration {i}/{times} -------------------------------")
            self.T = self.T_save
            self.iteration = 1
            self.cur_solution, self.cur_cost = self.initial_solution()
            self.anneal()
