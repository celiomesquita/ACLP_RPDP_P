import time
import multiprocessing as mp
import random
import math
import numpy as np
import copy
# from tabulate import tabulate
# table =[]
# row = []
# row.append(col1)
# row.append(col2)
# table.append(row)
# print( tabulate(table, headers=['col1','col2']) )

import common

# Process-based parallelism

ALPHA = 1 # pheromone exponent (linearly affects attractivity)
BETA  = 5 # heuristic exponent (exponentialy affects attractivity)
NANTS = 4 # number of ants per aircraft
ACFTS = NANTS # number of aircrafts (solutions)

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar):

    DeltaTau = 0.01
    if bestSoFar > 0:
         # at this point DeltaTau may be positive ou negative learning
        DeltaTau = (score - bestSoFar)/bestSoFar
    return DeltaTau


# Ant System (AS), the classic method that uses a random proportional state transition rule,
# while the pheromone is deposited by all ants proportionally to their solution quality and
# is evaporated in all the components. Each Ant makes use of "updateAntsField" to update 
# item attractiveness according to the its solution value.
def updateAntsField(score, bestSoFar, antsField, Pheromone, items, acft, ant):

    deltaTau = getDeltaTau(score, bestSoFar)

    maxPhero = 0.0

    # update pheromone level in all edges
    for id, phero in enumerate(Pheromone):

        #evaporate some pheromone 
        Pheromone[id] = math.sqrt(phero) / 1.2

        # update pheromone level
        if Pheromone[id] + deltaTau > 0:
            Pheromone[id] += deltaTau

        if Pheromone[id] > maxPhero:
            maxPhero = Pheromone[id]

        antsField[id] = Pheromone[id]**ALPHA * items[id].Attr**BETA
                
    print(f"{acft} \t {ant} \t {maxPhero:.3f} \t \t {deltaTau:.3f}")


def rouletteSelection(values): # at least 15 times faster than randomChoice
    s = sum(values)
    pick = random.uniform(0, s) # stop the roulette in a random point
    current = 0.
    for key, value in enumerate(values): # walk on the roulette
        current += value
        if current > pick:
            return key # return the roulette sector index
    return 0

# for tournament selection
class Value(object):
    def __init__(self):
        self.ID = -1
        self.V  = 0.0

# pick and delete an item from the neighborhood by a tournament selection
def selectItem(nbhood, values, pallet):

    # update values according to the "point of view" of this pallet
    maxTorque = 340. * 80. # kg * m, a heavy item on the ramp
    for j, it in enumerate(nbhood):
        thisTorque = it.W * abs(pallet.D)
        epsilon = thisTorque / maxTorque
        values[j] *= 2 - epsilon

    # make the tournament selection with 5 individuals
    individuals = [Value() for _ in range(5)]

    for i, _ in enumerate(individuals):
        individuals[i].ID = rouletteSelection(values)
        individuals[i].V  = values[individuals[i].ID]

    # choose the best individual
    individuals.sort(key=lambda x: x.V, reverse=False)
    j = individuals[0].ID

    item = nbhood[j]

    return item, j

def antSolve(pallets, items, cfg, k, secBreak, solTorque, solItems, antsField, Pheromone, lock, bestSoFar, acft, ant):

    nbhood   = [it for it in items]
    values   = [v  for v  in antsField]

    score = 0.0
    for i, _ in enumerate(pallets):
        score += pallets[i].PCS

    while nbhood:
        
        for i, p in enumerate(pallets):

            if len(nbhood) > 0:

                # pick from the neighborhood the probable best item for this pallet
                item, j = selectItem(nbhood, values, p)

                if pallets[i].isFeasible(item, 1.0, k, solTorque, solItems, cfg, lock):

                    pallets[i].putItem(item, solTorque, solItems, lock)

                    score += item.S

                    nbhood.pop(j)
                    values.pop(j)

                else:
                    if i == len(pallets)-1:
                        nbhood.pop(j)
                        values.pop(j)        

    updateAntsField(score, bestSoFar, antsField, Pheromone, items, acft, ant)

    return score
    

# def enqueue( antsQueue,     Gant, cfg, k):
#     antsQueue.put( antSolve(Gant, cfg, k) )

# def publish(antsQueue, procs, limit, antsField, pallets, items, cfg, k ):

#     for i, _ in enumerate(procs):

#         Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

#         # create a child process for each ant
#         procs[i] = mp.Process( target=enqueue, args=( antsQueue, Glocal, cfg, k ) ) # faster than thread
    
#         procs[i].start()
         

# def subscribe(antsQueue, procs, startTime, secBreak):

#     sols = [antsQueue.get() for _ in procs if time.perf_counter() - startTime < secBreak ]
#     for p in procs:
#         p.terminate()  
#     antsQueue.close()
#     return sols  


def Solve( pallets, items, cfg, k, limit, secBreak, solTorque, solItems ):

    numPallets = len(pallets)
    numItems   = len(items)

    # to control ants field of pheromone deposition and evaporation
    antsField = mp.Array('d', range(numItems))
    for j, _ in enumerate(antsField):
        antsField[j] = 0.5 # intermediate attractivity level    

    Pheromone = mp.Array('d', range(numItems))
    for j, _ in enumerate(Pheromone):
        Pheromone[j] = 0.5 # intermediate pheromone level 

    startTime = time.perf_counter()

    print("\nParallel Ant Colony Optimization for ACLP+RPDP")

    print(f"acft \t ant \t maxPhero \t deltaTau")

    limit = 0.75 # greedy volume limit %

    lock  = mp.Lock() # for use in parallel mode

    # sort pallets ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

    # aircrafts matrix      [columns]              [rows]
    aircrafts = [[p for p in pallets] for _ in np.arange(ACFTS)]

    bestAcft = -1
    for acft, pallets in enumerate(aircrafts):

        bestSoFar = 0.0

        # greedy phase
        for i, p in enumerate(aircrafts[acft]):
            common.fillPallet(aircrafts[acft][i], items, k, solTorque, solItems, cfg, lock, limit)
            bestSoFar += aircrafts[acft][i].PCS 

        # ants phase
        bestAnt = -1
        for ant in np.arange(NANTS):
            score = antSolve(aircrafts[acft], items, cfg, k, secBreak, solTorque, solItems, antsField, Pheromone, lock, bestSoFar, acft, ant)

            if score > bestSoFar:
                bestSoFar = score
                bestAnt   = ant
                bestAcft  = acft

    print(f"Best score so far: {bestSoFar} (best acft {bestAcft}) (best ant {bestAnt})")

    # --- mount solution matrix
    Z = np.zeros((numPallets,numItems))
    for j, i in enumerate(solItems):
        if i > -1: # alocated to some pallet
            Z[i][j] = 1

    return Z
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
