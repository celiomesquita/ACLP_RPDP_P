
import time
import math
import numpy as np
import methods as mno
import random

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = mno.NCPU

def rouletteSelection(values): # at least 15 times faster than randomChoice
    max = sum(values)
    pick = random.uniform(0, max)
    current = 0
    for key, value in enumerate(values):
        current += value
        if current > pick:
            return key

def randomChoice(values):
    sumVal = sum([v for v in values])
    probabilities = [v/sumVal for v    in           values ]
    indexes       = [i        for i, _ in enumerate(values)]
    return np.random.choice(indexes, p=probabilities)

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar, numAnts):
    if bestSoFar == 0:
        bestSoFar = 1000
    DeltaTau = (score - bestSoFar)/bestSoFar # at this point DeltaTau may be positive ou negative learning
    DeltaTau /= numAnts*numAnts
    return DeltaTau


# Ant System (AS), the classic method that uses a random proportional state transition rule, while the pheromone is deposited
# by all ants proportionally to their solution quality and is evaporated in all the components.
# each Ant makes use of "updatePheroAttract" to update pheromones
# and edge attractiveness according to the its solution value
def updatePheroAttract(score, bestSoFar, edges, numAnts):

    # evaporate some pheromone from all edges
    for id, e in enumerate(edges):
        edges[id].Pheromone = math.sqrt(e.Pheromone) / 1.25 # RHO = 0.2
        edges[id].updateAttract(ALPHA, BETA)

    deltaTau = getDeltaTau(score, bestSoFar, numAnts)

    maxAttract = 0.
    if deltaTau < 1:
        # update pheromone level in all edges
        for id, _ in enumerate(edges):
            edges[id].Pheromone += deltaTau
            if edges[id].Pheromone < 0.:
                edges[id].Pheromone = 0.
            edges[id].updateAttract(ALPHA, BETA)

            if edges[id].Attract > maxAttract:
                maxAttract = edges[id].Attract

        for id, _ in enumerate(edges):
            edges[id].Attract /= maxAttract


# pick and delete an edge from the neighborhood by a proportional roulette wheel
def pickFromNbhood(nbhood, values):
    i = rouletteSelection(values)
    e = nbhood[i]
    nbhood.pop(i)
    values.pop(i)
    return e

def Solve( pallets, items, startTime, cfg, k, limit):  # items include kept on board

    SEC_BREAK = NANTS

    print("\nAnt Colony Optimization for ACLP+RPDP")

    antsField = mno.mountEdges(pallets, items, cfg)

    # initialize the best solution so far           1.0 totally greedy
    Gbest = mno.Solution(antsField, pallets, items, 1.00, cfg, k)

    initialS = Gbest.S
    print(f"Initial score = {initialS}")

    numPallets = len(pallets)
    numItems   = len(items)
    numAnts = 0
    stagnant = 0
    improvements = 0

    while stagnant <= 3:# and (time.perf_counter() - startTime) < SEC_BREAK:

        Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

        deltaTau = getDeltaTau(Glocal.S, initialS, NANTS)

        for _ in np.arange(NANTS):

            if (time.perf_counter() - startTime) > SEC_BREAK:
                break

            numAnts += 1
            
            Gant = Glocal

            Nbhood   = [ce for ce in Gant.Edges if not ce.InSol]
            attracts = [ce.Attract for ce in Nbhood]

            while Nbhood:
                
                ce = pickFromNbhood(Nbhood, attracts)

                if Gant.isFeasible(ce, 1.0, cfg, k):
                    antsField[ce.ID].Pheromone += deltaTau
                    Gant.putInSol(ce)
                else:
                    antsField[ce.ID].Pheromone -= deltaTau

                antsField[ce.ID].updateAttract(ALPHA, BETA)          

            if Gant.S > Glocal.S:
                Glocal = Gant

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
            improvements += 1
        else:
            stagnant += 1

        updatePheroAttract(Glocal.S, Gbest.S, antsField, NANTS)


    if initialS == 0:
        initialS = 1000

    print(f"Used {numAnts} ants | ratio {Glocal.S/initialS:.3f} | {improvements} improvements")

    return mno.getSolMatrix(Gbest.Edges, numPallets, numItems)

        
if __name__ == "__main__":

    values = [1,2,3,4,5,6,7,8,9]

    sums = [0 for _ in values]

    t0 = time.perf_counter()

    for v in range(1000):
        i = randomChoice(values)
        sums[i] += 1

    print(sums)

    t1 = time.perf_counter()

    sums = [0 for _ in values]

    for v in range(1000):
        i = rouletteSelection(values)
        sums[i] += 1

    print(sums)

    t2 = time.perf_counter()

    print(f"{(t1-t0)/(t2-t1)}")

    # print("----- Please execute module main -----")


