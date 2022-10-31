
from time import time
import math
import numpy as np
import methods as mcuda

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = 4   # number of ants for team

# For ACO proportional selection
RNG = np.random.default_rng()

# Proportional Roulette Selection (biased if greediness > 0)
def rouletteSelection(values, sumVal, greediness):

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

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
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
def pickFromNbhood(nbhood, values, sumVal, greediness):
    i = rouletteSelection(values, sumVal, greediness)
    e = nbhood[i]
    nbhood.pop(i)
    values.pop(i)
    sumVal -= e.Attract
    return e

def Solve( pallets, items, startTime, cfg, k):  # items include kept on board

    print("\nAnt Colony Optimization for ACLP+RPDP")

    antsField = mcuda.mountEdges(pallets, items, cfg, k)
    Nbhood    = mcuda.edges_copy(antsField)

    # initialize the best solution                 1.0 totally greedy
    Gbest = mcuda.Solution(Nbhood, pallets, items, 1.0, cfg, k)

    numPallets = len(pallets)
    numItems   = len(items)
 
    stagnant = 0
    numAnts = 0
    while stagnant <= 3:# and (time() - startTime) < mcuda.SEC_BREAK:

        Glocal = Gbest
   
        for _ in np.arange(NANTS): # sequential ants

            # if (time() - startTime) > mcuda.SEC_BREAK:
            #     break
        
            numAnts += 1
            
            Nbhood = mcuda.edges_copy(antsField)

            # initilize a solution for each ant
            Gant = mcuda.Solution(Nbhood, pallets, items, 0.9, cfg, k)

            # antsField edges are a heritage from previous ants
            # prepare for the random proportional selection in pickFromNbhood method
            attracts = [ce.Attract for ce in antsField]
            attractSum = sum(attracts, key=lambda x: x.Attract)

            # builds or completes a solution by exploring the entire items neighborhood
            while Gant.Nbhood:

                # if ((time() - startTime) > mcuda.SEC_BREAK):
                #     break                

                # pick a candidate edge from the neighborhood by a random proportional selection
                # the last argument is the greediness
                ce = pickFromNbhood(Gant.Nbhood, attracts, attractSum, 0.0)

                if Gant.isFeasible(ce, 1.00, cfg, k):
                    Gant.AppendEdge(ce)
                    
            if Gant.S > Glocal.S:
                Glocal = Gant

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
            updatePheroAttract(Gant.S, Gbest.S, antsField, numAnts)
        else:
            stagnant += 1

    print(f"Number of ants used: {numAnts}")

    return mcuda.getSolMatrix(Gbest.Edges, numPallets, numItems)


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")