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

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestScore):

    # at this point DeltaTau may be positive ou negative learning
    DeltaTau = (score - bestScore)/bestScore
    # return DeltaTau + random.uniform(0, 1)
    return DeltaTau


# Ant System (AS), the classic method that uses a random proportional state transition rule,
# while the pheromone is deposited by all ants proportionally to their solution quality and
# is evaporated in all the components. Each Ant makes use of "updateAntsField" to update 
# item attractiveness according to the its solution value.
def updateAntsField(score, bestScore, Attract, Phero, items, lock):

    deltaTau = getDeltaTau(score, bestScore)

    with lock:

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

def antSolve(pallets, items, cfg, k, secBreak, solTorque, solItems, Attract, Phero, lock, bestScore, stat, accum, maxD):

    nbhood   = [it for it in items]
    values   = [v  for v  in Attract]

    maxTorque = max([it.W * maxD for it in nbhood])

    # accum["score"]  -> accumulated score
    # accum["volume"] -> accumulated volume

    while nbhood:

        for i, p in enumerate(pallets):
            
            if len(nbhood) > 0:

                # pick from the neighborhood the probable best item for this pallet
                item, j = selectItem(nbhood, values, p, maxTorque)

                if pallets[i].isFeasible(item, 1.0, k, solTorque, solItems, lock, cfg ):

                    pallets[i].putItem(item, solTorque, solItems, lock)

                    accum["score"]  += item.S
                    accum["volume"] += item.V

                    nbhood.pop(j) # pop if included in solution
                    values.pop(j)

                else: # pop in the last pallet tested
                    if i == len(pallets)-1:
                        nbhood.pop(j)
                        values.pop(j)        

    updateAntsField(accum["score"], bestScore, Attract, Phero, items, lock)

    if stat:
        AttractVar  = statistics.variance(Attract)
        PheroVar    = statistics.variance(Phero)
        AttractMean = statistics.mean(Attract) 
        PheroMean   = statistics.mean(Phero)    

        print(f"{PheroVar:.2f}\t\t{PheroMean:.2f}\t\t{AttractVar:.2f}\t\t{AttractMean:.2f}")

    accum["volume"] /= cfg.volCap

def Solve( pallets, items, cfg, k, limit, secBreak, mode, solTorque, solItems ):

    # to control the general attractiveness for the tournament selection
    Attract = mp.Array('d', np.arange(len(items)))
    for j, _ in enumerate(Attract):
        Attract[j] = 0.5

    # to control pheromone deposition and evaporation
    Phero = mp.Array('d', np.arange(len(items)))
    for j, _ in enumerate(Phero):
        Phero[j] = 0.5

    startTime = time.perf_counter()

    limit = 0.75 # ACO greedy limit

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    print(f"\n{mode} Ant Colony Optimization for ACLP+RPDP\n")        

    stat = False # True to show statistics

    if stat:
        print(f"PheroVar\tPheroMean\tAttractVar\tAttractMean")

    lock  = mp.Lock() # for use in parallel mode


    # greedy serial phase (both for serial and parallel modes)

    initScore   = 0.0 # best score so far
    initVolume  = 0.0 # best volume so far
    maxD        = 0.0 # the maximum distance from the CG
    initPallets = copy.deepcopy(pallets)
    initPallets.sort(key=lambda x: abs(x.D)) # sort pallets ascendent by CG distance

    for i, _ in enumerate(initPallets):               
        common.fillPallet(initPallets[i], items, k, solTorque, solItems, lock, cfg, limit)
        initScore  += initPallets[i].PCS
        initVolume += initPallets[i].PCV   
        if abs(initPallets[i].D) > maxD:
            maxD = abs(initPallets[i].D)

    # accum = dict(score=initScore, volume=initVolume)

    bestScore  = initScore  # best score so far
    bestVolume = initVolume # best volume so far

    # ACO phase
    bestIter = 0
    iter = 0
    stagnant = 0

    numAnts  = 1 # number of ants per iteration
    if mode == "Parallel":
        numAnts = 10

    while stagnant < 1 and (time.perf_counter() - startTime) < secBreak: # iterations      

        ants       = [None for _ in np.arange(numAnts)]
        accums     = [None for _ in np.arange(numAnts)] # for ant scores and volumes
        antPallets = [None for _ in np.arange(numAnts)]
        antTorque  = [None for _ in np.arange(numAnts)]
        antItems   = [None for _ in np.arange(numAnts)]

        bestAnt = 0
        bestAntScore  = initScore
        bestAntVolume = initVolume

        # parallel phase: initialize ants
        for a, _ in enumerate(ants): # ants
          
            antPallets[a] = copy.deepcopy(initPallets)
            antTorque[a]  = solTorque
            antItems[a]   = solItems
            accums[a]     = dict(score=initScore, volume=initVolume)

            ants[a] = mp.Process( target=antSolve, args=( antPallets[a], items, cfg, k, secBreak,\
                antTorque[a], antItems[a], Attract, Phero, lock, bestScore, stat, accums[a], maxD) )

            ants[a].start() # send ant
        
        # wait until time limit: secBreak
        while time.perf_counter() - startTime <= secBreak:
            if not any(p.is_alive() for p in ants):
                print(f"All the ants ({numAnts}) of iteration {iter} have ended.")
                break
        else:
            # We only enter this if we didn't 'break' above.
            print("timed out, killing all ants")
            for p in ants:
                p.terminate()
                p.join()

        # look for the best ant solution
        for a, _ in enumerate(ants):
            if accums[a]["score"] > bestAntScore or accums[a]["volume"] > bestAntVolume:
                bestAntScore  = accums[a]["score"] 
                bestAntVolume = accums[a]["volume"] 
                bestAnt = a

        if bestAntScore > bestScore or bestAntVolume > bestVolume:
            bestScore  = bestAntScore
            bestVolume = bestAntVolume
            bestIter   = iter
            pallets    = copy.deepcopy(antPallets[bestAnt])
            solTorque  = antTorque[bestAnt] 
            solItems   = antItems[bestAnt]
            stagnant = 0
        else:
            stagnant += 1

        iter += 1

    print(f"Best iteration {bestIter}/{iter}.")
    if mode == "Parallel":
        print(f"Best ant {bestAnt}/{numAnts}.")

        
if __name__ == "__main__":

    print("----- Please execute module main -----")
