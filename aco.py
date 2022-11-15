
import time
import math
import numpy as np
import methods as mno
import random
from scipy import stats

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = 6

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
def getDeltaTau(score, bestSoFar):

    DeltaTau = (score - bestSoFar)/bestSoFar # at this point DeltaTau may be positive ou negative learning
    return DeltaTau*30.


# Ant System (AS), the classic method that uses a random proportional state transition rule, while the pheromone is deposited
# by all ants proportionally to their solution quality and is evaporated in all the components.
# each Ant makes use of "updatePheroAttract" to update pheromones
# and edge attractiveness according to the its solution value
def updatePheroAttract(score, bestSoFar, edges):

    deltaTau = getDeltaTau(score, bestSoFar)

    Pheromone = [] 

    # update pheromone level in all edges
    for id, e in enumerate(edges):

        #evaporate some pheromone 
        edges[id].Pheromone = math.sqrt(e.Pheromone) / 1.4 # RHO = 0.2

        # update pheromone level
        if e.Pheromone + deltaTau > 0 and e.Pheromone + deltaTau <= 1:
            edges[id].Pheromone += deltaTau
        
        Pheromone.append(e.Pheromone)
        
        # update the general attractiveness
        edges[id].updateAttract(ALPHA, BETA)

    # mean = stats.tmean(Pheromone)
    # sdev = stats.tstd(Pheromone)
    # print(f"Mean: {mean:.2f} | StdDev: {sdev:.2f}")

# pick and delete an edge from the neighborhood by a proportional roulette wheel
def pickFromNbhood(nbhood, values):
    i = rouletteSelection(values)
    e = nbhood[i]
    nbhood.pop(i)
    values.pop(i)
    return e

def Solve( pallets, items, startTime, cfg, k, limit, secBreak):  # items include kept on board

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

    while stagnant <= 3 and (time.perf_counter() - startTime) < secBreak:

        Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

        for _ in np.arange(NANTS):

            if (time.perf_counter() - startTime) > secBreak:
                break

            numAnts += 1
            
            Gant = Glocal

            Nbhood   = [ce for ce in Gant.Edges if not ce.InSol]
            attracts = [ce.Attract for ce in Nbhood]

            while Nbhood:
                
                ce = pickFromNbhood(Nbhood, attracts)

                if Gant.isFeasible(ce, 1.0, cfg, k):
                    Gant.putInSol(ce)
       
            if Gant.S > Glocal.S:
                Glocal = Gant

            updatePheroAttract(Glocal.S, Gbest.S, antsField)

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
            improvements += 1
        else:
            stagnant += 1

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


