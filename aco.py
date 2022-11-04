
import time
import math
import numpy as np
import methods as mno

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = 5   # number of ants for team

def rouletteSelection(values):
    sumVal = sum([v for v in values])
    probabilities = [v/sumVal for v    in           values ]
    indexes       = [i        for i, _ in enumerate(values)]
    return np.random.choice(indexes, p=probabilities)

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar, numAnts):
    DeltaTau = (score - bestSoFar)/bestSoFar # at this point DeltaTau may be positive ou negative learning
    DeltaTau /= numAnts*numAnts
    return DeltaTau

# each Ant makes use of "updatePheroAttract" to update pheromones
# and edge attractiveness according to the its solution value
def updatePheroAttract(score, bestSoFar, edges, numAnts, reset=False):

    # if True resets pheromone and attractiveness levels to diversify the search
    if reset:
        for i, e in enumerate(edges):
            edges[i].Pheromone = 0.5
            edges[i].updateAttract(ALPHA, BETA)
        return      

    # evaporate some pheromone from all edges
    for id, e in enumerate(edges):
            # flatten pheromone curve
        edges[id].Pheromone = math.sqrt(e.Pheromone) / 1.25 # RHO = 0.2
        edges[id].updateAttract(ALPHA, BETA)

    deltaTau = getDeltaTau(score, bestSoFar, numAnts)

    if deltaTau < 1:
        # update pheromone level in all edges
        for id, e in enumerate(edges):
            edges[id].Pheromone += deltaTau
            if edges[id].Pheromone < 0.:
                edges[id].Pheromone = 0.
            edges[id].updateAttract(ALPHA, BETA)

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

    print(f"Initial score = {Gbest.S}")

    numPallets = len(pallets)
    numItems   = len(items)
    numAnts = 0
    stagnant = 0
    improvements = 0

    while stagnant <= 5:# and (time.perf_counter() - startTime) < SEC_BREAK:

        Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

        for _ in np.arange(NANTS):

            # if (time.perf_counter() - startTime) > SEC_BREAK:
            #     break

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

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
            improvements += 1
        else:
            stagnant += 1

        updatePheroAttract(Glocal.S, Gbest.S, antsField, NANTS)


    print(f"Used {numAnts} ants | stagnated {stagnant-1} times | {improvements} improvements")

    return mno.getSolMatrix(Gbest.Edges, numPallets, numItems)

        
if __name__ == "__main__":

    print("----- Please execute module main -----")


