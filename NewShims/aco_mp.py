import time
import multiprocessing as mp
import random
import math
import numpy as np
import statistics
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

ALPHA = 1 # pheromone exponent
BETA  = 1 # heuristic exponent
NANTS = 10 # number of ants per aircraft
ACFTS = NANTS # number of aircrafts (solutions)

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar):

    # at this point DeltaTau may be positive ou negative learning
    DeltaTau = (score - bestSoFar)/bestSoFar
    # return DeltaTau + random.uniform(0, 1)
    return DeltaTau


# Ant System (AS), the classic method that uses a random proportional state transition rule,
# while the pheromone is deposited by all ants proportionally to their solution quality and
# is evaporated in all the components. Each Ant makes use of "updateAntsField" to update 
# item attractiveness according to the its solution value.
def updateAntsField(score, bestSoFar, Attract, Phero, items):

    deltaTau = getDeltaTau(score, bestSoFar)

    # update pheromone level in all edges
    for id, phero in enumerate(Phero):

        #evaporate some pheromone 
        Phero[id] = math.sqrt(phero) / 1.35

        # update pheromone level
        if Phero[id] + deltaTau > 0 and  Phero[id] + deltaTau < 1:
            Phero[id] += deltaTau

        Attract[id] = Phero[id]**ALPHA * items[id].Attr**BETA


# at least 15 times faster than randomChoice
def rouletteSelection(values): 
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
def selectItem(nbhood, values, pallet, maxTorque):

    for j, it in enumerate(nbhood):
        thisTorque = it.W * abs(pallet.D)
        epsilon = thisTorque / maxTorque
        values[j] *= 2 - epsilon
      
    # make the tournament selection with 3 individuals
    individuals = [Value() for _ in range(3)]

    for i, _ in enumerate(individuals):
        individuals[i].ID = rouletteSelection(values)
        individuals[i].V  = values[individuals[i].ID]

    # choose the best individual
    individuals.sort(key=lambda x: x.V, reverse=True)
    j = individuals[0].ID

    item = nbhood[j]

    return item, j

def antSolve(pallets, items, cfg, k, secBreak, solTorque, solItems, Attract, Phero, lock, bestSoFar):

    nbhood   = [it for it in items]
    values   = [v  for v  in Attract]

    volume = 0.0
    score = 0.0
    maxD = 0.0 # the maximum distance from the CG
    for i, p in enumerate(pallets):
        score  += pallets[i].PCS
        volume += pallets[i].PCV
        if abs(p.D) > maxD:
            maxD = abs(p.D)

    maxTorque = max([it.W * maxD for it in nbhood])

    while nbhood:

        for i, p in enumerate(pallets):
            
            if len(nbhood) > 0:

                # pick from the neighborhood the probable best item for this pallet
                item, j = selectItem(nbhood, values, p, maxTorque)

                if pallets[i].isFeasible(item, 1.0, k, solTorque, solItems, cfg, lock):

                    pallets[i].putItem(item, solTorque, solItems, lock)

                    score  += item.S
                    volume += item.V

                    nbhood.pop(j) # pop if included in solution
                    values.pop(j)

                else: # pop in the last pallet tested
                    if i == len(pallets)-1:
                        nbhood.pop(j)
                        values.pop(j)        

    updateAntsField(score, bestSoFar, Attract, Phero, items)

    AttractVar  = statistics.variance(Attract)
    PheroVar    = statistics.variance(Phero)
    AttractMean = statistics.mean(Attract) 
    PheroMean   = statistics.mean(Phero)    

    print(f"{PheroVar:.2f}\t\t{PheroMean:.2f}\t\t{AttractVar:.2f}\t\t{AttractMean:.2f}")


    return score, volume/cfg.volCap
    

# def enqueue( antsQueue,     Gant, cfg, k):
#     antsQueue.put( antSolve(Gant, cfg, k) )

# def publish(antsQueue, procs, limit, Attract, pallets, items, cfg, k ):

#     for i, _ in enumerate(procs):

#         Glocal = mno.Solution(Attract, pallets, items, limit, cfg, k)

#         # create a child process for each ant
#         procs[i] = mp.Process( target=enqueue, args=( antsQueue, Glocal, cfg, k ) ) # faster than thread
    
#         procs[i].start()
         

# def subscribe(antsQueue, procs, startTime, secBreak):

#     sols = [antsQueue.get() for _ in procs if time.perf_counter() - startTime < secBreak ]
#     for p in procs:
#         p.terminate()  
#     antsQueue.close()
#     return sols  


def Solve( pallets, items, cfg, k, limit, secBreak, a, solTorque, solItems ):

    numPallets = len(pallets)
    numItems   = len(items)

    # to control the general attractiveness for the tournament selection
    Attract = mp.Array('d', range(numItems))

    # to control pheromone deposition and evaporation
    Phero = mp.Array('d', range(numItems))

    startTime = time.perf_counter()

    print("\nParallel Ant Colony Optimization for ACLP+RPDP")

    print(f"PheroVar\tPheroMean\tAttractVar\tAttractMean")

    lock  = mp.Lock() # for use in parallel mode

    # sort pallets ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D))

    bestSoFar = 0.0

    # greedy phase
    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, solTorque, solItems, cfg, lock, limit)
        bestSoFar += pallets[i].PCS

    # ACO phase
    bestAnt = -1
    bestAntItems = solItems
    bestVolume = 0.0    
    for ant in np.arange(NANTS):

        antPallets = copy.deepcopy(pallets)
        antTorque  = solTorque
        antItems   = solItems

        score, volume = antSolve(antPallets, items, cfg, k, secBreak, antTorque, antItems, Attract, Phero, lock, bestSoFar)

        if score > bestSoFar or volume > bestVolume:
            bestSoFar = score
            bestAnt   = ant
            bestAntItems = antItems
            bestVolume = volume

    endtTime = time.perf_counter()

    print(f"Best ant ({bestAnt}): score {bestSoFar} | volume {bestVolume:.2f} | elapsed {endtTime-startTime:.2f}")

    # --- mount solution matrix
    Z = np.zeros((numPallets,numItems))
    for j, i in enumerate(bestAntItems):
        if i > -1: # alocated to some pallet
            Z[i][j] = 1

    return Z
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
