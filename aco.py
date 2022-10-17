
from time import time
import math
import numpy as np
import methods
from multiprocessing import Queue, Process

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = 4   # number of ants for team

TEAMS = 7 # ants teams, normally the number of processors

# For ACO proportional selection
RNG = np.random.default_rng()

# Proportional Roulette Selection (biased if greediness > 0)
def select(values, sumVal, greediness):

    n = len(values)
    if n == 0:
        return -1
    if n == 1:
        return 0

    threshold = RNG.random()*(1.0-greediness) + greediness
    threshold *= sumVal

    pointer = 0.0
    for j, v in enumerate(values):
        pointer += v
        if pointer >= threshold: # when pointer reaches the threshold
            return j # returns the chosen index

    # just in case ....
    return int(n * RNG.random())

# calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar, numAnts):

    if bestSoFar == 0:
        return 0

    if score > bestSoFar: # better the score more pheromone is deposited
        score = bestSoFar

    dt = score / bestSoFar

    if numAnts < NANTS:
        numAnts = NANTS

    dt /= numAnts

    return dt

# each Ant makes use of "updatePheroAttract" to update pheromones
# and edge attractiveness according to the its solution value
def updatePheroAttract(score, bestSoFar, edges, numAnts):

    # evaporate some pheromone from all edges
    for id, e in enumerate(edges):
        # flatten pheromone curve
        edges[id].Pheromone = math.sqrt(e.Pheromone) / 1.25 # RHO = 0.2
        edges[id].updateAttract(ALPHA, BETA)

    deltaTau = getDeltaTau(score, bestSoFar, numAnts)

    if deltaTau < 1:
        # deposit some pheromone in the edges
        for id, e in enumerate(edges):
            if e.Pheromone < 1.0:
                edges[id].Pheromone += deltaTau
                edges[id].updateAttract(ALPHA, BETA)

# pick and delete an edge from the neighborhood by a proportional roulette wheel
# def selectInNbhood(nbhood, values, sumVal, greediness):
#     i = select(values, sumVal, greediness)
#     e = nbhood[i]
#     nbhood.pop(i)
#     values.pop(i)
#     return e, sumVal - e.Attract

def selectInNbhood(nbhood, values, sumVal, greediness):
    i = select(values, sumVal, greediness)
    e = nbhood[i]
    nbhood[i].Attract = 0
    values[i] = 0
    return e, sumVal - e.Attract    

def Solve( pallets, items, startTime, cfg, k):  # items include kept on board

    print("\nAnt Colony Optimization for ACLP+RPDP")

    edges = methods.mountEdges(pallets, items, cfg, k)

    # initialize the best solution                  1.0 totally greedy
    Gbest = methods.Solution(edges, pallets, items, 1.0, cfg, k)
 
    stagnant = 0
    numAnts = 0
    while stagnant < 1 and (time() - startTime) < methods.SEC_BREAK:

        Glocal = Gbest
   
        for _ in np.arange(NANTS): # sequential ants

            if (time() - startTime) > methods.SEC_BREAK:
                break
        
            numAnts += 1

            # initilize a solution for each ant
            Gant = methods.Solution(edges, pallets, items, 0.9, cfg, k, False)

            # neighborhood edges are a heritage from previous ants
            nbhood = np.zeros(len(edges))
            # prepare for the random proportional selection in selectInNbhood method
            attracts = np.zeros(len(edges))

            attractSum = 0.0
            numNb = 0
            for ce in Gant.Edges:
                if ce.Item.P == -1: # items only
                    attracts[ce.ID] = ce.Attract
                    nbhood[ce.ID] = ce
                    attractSum += ce.Attract
                    numNb += 1
                else:
                    attracts[ce.ID] = 0
                    nbhood[ce.ID].Attract = 0   


            # builds or completes a solution by exploring the entire items neighborhood
            while numNb:

                if ((time() - startTime) > methods.SEC_BREAK):
                    break                

                # pick a candidate edge from the neighborhood by a random proportional selection
                # the last argument is the greediness
                ce, attractSum = selectInNbhood(nbhood, attracts, attractSum, 0.0)

                if Gant.isFeasible(ce, 1.00, cfg, k):
                    Gant.includeEdge(ce)
                    
            if Gant.S > Glocal.S:
                Glocal = Gant

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
            updatePheroAttract(Gant.S, Gbest.S, nbhood, numAnts)
        else:
            stagnant += 1

    print(f"Number of ants used: {numAnts}")

    # decision matrix for which items will be put in which pallet
    X = np.zeros((len(pallets),len(items))) # needs 2 parenthesis
    for e in Gbest.Edges:
        if e.InSol == 1:
            X[e.Pallet.ID][e.Item.ID] = 1

    return X

# A team of ants is a process, a set of ants that will solve the problem sequentially,
# each team in its own process running concurrently.
class AntsTeam():
    def __init__(self):
        self.teams = []
        self.solsQueue = Queue()

    # triggers build and put new solutions and modified edges in queues
    @staticmethod
    def _wrapper(func, solsQueue, edges, pallets, items, numSteps, startTime, cfg, k):

        sol = func(edges, pallets, items, numSteps, startTime, cfg, k)

        solsQueue.put(sol)

    # start an ant - a new process
    def Start(self, func, edges, pallets, items, numSteps, startTime, cfg, k):

        args_array = [func, self.solsQueue, edges, pallets, items, numSteps, startTime, cfg, k]

        team = Process(target=self._wrapper, args=args_array)

        self.teams.append(team)

        team.start()

    def Wait(self):
        """wait for all ants to finish and send back results"""
        sols_set = []

        # pick from the queues new sols and modified edges sets
        for _ in self.teams:

            sol = self.solsQueue.get()
            sols_set.append(sol)

        # wait for all ants to finish
        for team in self.teams:
            team.join()

        # send back new orgs and modified edges sets
        return sols_set


def teamSolve(pallets, items, startTime, cfg, k):

    # start to solve with ACO
    prevBest = 0

    # initialize the edges between pallets and items
    edges = [None] * len(pallets) * len(items)
    id = 0
    for p in pallets:
        for it in items:
            edges[id] = methods.Edge(id, p, it, cfg)
            id += 1
    numSteps = id # for pheromone droplet size

    team = AntsTeam()

    for _ in range(TEAMS):

        if (time() - startTime) > methods.SEC_BREAK:
            break        

        team.Start( sendAnts, edges, pallets, items, numSteps, startTime, cfg, k )

    sols_set = team.Wait()  # get teams solutions

    best = 0
    for i, sol in enumerate(sols_set):
        if sol.S > prevBest:  # if improved
            prevBest = sol.S
            best = i
    
    # decision matrix for which items will be put in which pallet
    X = np.zeros((len(pallets),len(items))) # needs 2 parenthesis
    for e in sols_set[best].Edges:
        if e.InSol == 1:
            X[e.Pallet.ID][e.Item.ID] = 1

    return X


def Build( sol, bestSol, edges, numSteps, numAnts, cfg, k ):

    # neighborhood edges are a heritage from previous ants
    nbhood = np.zeros(len(edges))
    # prepare for the random proportional selection in selectInNbhood method
    attracts = np.zeros(len(edges))
    attractSum = 0.0

    numNb = 0
    for ce in edges:
        if ce.Item.P == -1: # items only
            attracts[ce.ID] = ce.Attract
            nbhood[ce.ID] = ce
            attractSum += ce.Attract
            numNb += 1
        else:
            attracts[ce.ID] = 0
            nbhood[ce.ID].Attract = 0           

    # builds or completes a solution by exploring the entire items neighborhood
    while numNb:

        numNb -= 1

        # pick a candidate edge from the neighborhood by a random proportional selection
        # the last argument is the greediness
        ce, attractSum = selectInNbhood(nbhood, attracts, attractSum, 0.0)

        #                     volume limit extension: 1.0 no extension
        if sol.isFeasible(ce, 1.00, cfg, k):

            sol.includeEdge(ce)

            # local pheromone update - evaporate to deceit other ants
            deltaTau = getDeltaTau(sol.S, bestSol.S, numAnts)
            if ce.Pheromone > deltaTau:
                edges[ce.ID].Pheromone -= deltaTau
                edges[ce.ID].updateAttract(ALPHA, BETA)

    # ant system - update in all edges
    updatePheroAttract(sol.S, bestSol.S, edges, numAnts)

    if sol.S > bestSol.S:
        print(f"----- improved: {sol.S} -----")
        return sol
    return bestSol

def sendAnts( edges, pallets, items, numSteps, startTime, cfg, k):

    # an initial greedy solution
    bestSol = methods.Solution(edges, pallets, items, 1.0, cfg, k)

    # limit = 1.0 - 3200/float(len(edges)) # a new try with a greedy ACO
    limit = 0.85

    for _ in np.arange(NANTS):  # Ants solving sequentially

        elapsed = (time() - startTime)/60
        if elapsed > methods.SEC_BREAK: # break time in minutes
            break

        # initilize an empty or greedy solution
        sol = methods.Solution(edges, pallets, items, limit, cfg, k)

        # bestSol and edges are inherited from previous ants
        bestSol = Build( sol, bestSol, edges, numSteps, NANTS, cfg, k )

    return bestSol

if __name__ == "__main__":

    print("----- Please execute the main py file -------------")