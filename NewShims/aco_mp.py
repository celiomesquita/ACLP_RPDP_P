import time
import multiprocessing as mp
import random
import math
import numpy as np

import common

# Process-based parallelism

ALPHA = 1 # pheromone exponent (linearly affects attractivity)
BETA  = 5 # heuristic exponent (exponentialy affects attractivity)
NANTS = 6 # number of ants per aircraft
ACFTS = 5 # number of aircrafts (solutions)

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
def updateAntsField(score, bestSoFar, antsField, items):

    deltaTau = getDeltaTau(score, bestSoFar)

    maxPhero = 0.0

    # update pheromone level in all edges
    for id, phero in enumerate(antsField):

        #evaporate some pheromone 
        antsField[id] = math.sqrt(phero) / 1.4 # RHO = 0.2

        # update pheromone level
        if antsField[id] + deltaTau > 0:
            antsField[id] += deltaTau

        if antsField[id] > maxPhero:
            maxPhero = antsField[id]
                
    print(f"maxPhero = {maxPhero:.3f} \t deltaTau = {deltaTau:.3f}")

    # maxAttr = 0.0

    for id, phero in enumerate(antsField):

        antsField[id] /= maxPhero

        # update the general attractiveness
        antsField[id] = antsField[id]**ALPHA * items[id].Attr**BETA

    #     if items[id].Attr > maxAttr:
    #         maxAttr = items[id].Attr
    # print(f"maxAttr = {maxAttr:.3f}")

def rouletteSelection(values): # at least 15 times faster than randomChoice
    s = sum(values)
    pick = random.uniform(0, s)
    current = 0.
    for key, value in enumerate(values):
        current += value
        if current > pick:
            return key
    return 0


# pick and delete an item from the neighborhood by a proportional roulette wheel
def pickFromNbhood(nbhood, values):

    j = rouletteSelection(values)
    item = nbhood[j]
    nbhood.pop(j)
    values.pop(j)
    return item

def antSolve(pallets, items, cfg, k, secBreak, solTorque, solItems, antsField, lock, bestSoFar):

    nbhood   = [it for it in items]
    values   = [v  for v  in antsField]

    score = 0.0
    for i, _ in enumerate(pallets):
        score += pallets[i].PCS

    while nbhood:
        
        item = pickFromNbhood(nbhood, values)

        for i, _ in enumerate(pallets):

            if pallets[i].isFeasible(item, 1.0, k, solTorque, solItems, cfg, lock):

                pallets[i].putItem(item, solTorque, solItems, lock)

                score += item.S

    updateAntsField(score, bestSoFar, antsField, items)

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


def Solve( pallets, items, cfg, k, limit, secBreak, solTorque, solItems, antsField ):

    numPallets = len(pallets)
    numItems   = len(items)

    startTime = time.perf_counter()

    print("\nParallel Ant Colony Optimization for ACLP+RPDP")

    limit = 0.95

    lock  = mp.Lock()

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

    aircrafts = [None for _ in np.arange(ACFTS)]

    for a, _ in enumerate(aircrafts):

        aircrafts[a] = pallets

        bestSoFar = 0.0
        for i, _ in enumerate(aircrafts[a]):
            common.fillPallet(aircrafts[a][i], items, k, solTorque, solItems, cfg, lock, limit)
            bestSoFar += aircrafts[a][i].PCS 

        score = antSolve(aircrafts[a], items, cfg, k, secBreak, solTorque, solItems, antsField, lock, bestSoFar)

        if score > bestSoFar:
            bestSoFar = score

    print(f"Best score so far: {bestSoFar}")

    # --- mount solution matrix
    Z = np.zeros((numPallets,numItems))
    for j, i in enumerate(solItems):
        if i > -1: # alocated to some pallet
            Z[i][j] = 1

    return Z
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
