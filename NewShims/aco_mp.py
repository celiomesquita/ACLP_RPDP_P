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
NANTS = 1 # number of ants per iteration

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

def antSolve(pallets, items, cfg, k, secBreak, solTorque, solItems, Attract, Phero, lock, bestSoFar, stat, thisdict):

    nbhood   = [it for it in items]
    values   = [v  for v  in Attract]

    maxTorque = max([it.W * thisdict["maxD"] for it in nbhood])

    while nbhood:

        for i, p in enumerate(pallets):
            
            if len(nbhood) > 0:

                # pick from the neighborhood the probable best item for this pallet
                item, j = selectItem(nbhood, values, p, maxTorque)

                if pallets[i].isFeasible(item, 1.0, k, solTorque, solItems, lock, cfg ):

                    pallets[i].putItem(item, solTorque, solItems, lock)

                    thisdict["score"]  += item.S
                    thisdict["volume"] += item.V

                    nbhood.pop(j) # pop if included in solution
                    values.pop(j)

                else: # pop in the last pallet tested
                    if i == len(pallets)-1:
                        nbhood.pop(j)
                        values.pop(j)        

    updateAntsField(thisdict["score"], bestSoFar, Attract, Phero, items)

    if stat:
        AttractVar  = statistics.variance(Attract)
        PheroVar    = statistics.variance(Phero)
        AttractMean = statistics.mean(Attract) 
        PheroMean   = statistics.mean(Phero)    

        print(f"{PheroVar:.2f}\t\t{PheroMean:.2f}\t\t{AttractVar:.2f}\t\t{AttractMean:.2f}")

    thisdict["volume"] /= cfg.volCap

def Solve( pallets, items, cfg, k, limit, secBreak, mode, solTorque, solItems ):

    # to control the general attractiveness for the tournament selection
    Attract = mp.Array('d', np.arange(len(items)))

    # to control pheromone deposition and evaporation
    Phero = mp.Array('d', np.arange(len(items)))

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

    # sort pallets ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D))

    bestSoFar  = 0.0
    bestVolume = 0.0
    maxD       = 0.0 # the maximum distance from the CG

    # greedy serial phase (both for serial and parallel modes)
    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, solTorque, solItems, lock, cfg, limit)
        bestSoFar  += pallets[i].PCS
        bestVolume += pallets[i].PCV    
        if abs(pallets[i].D) > maxD:
            maxD = abs(pallets[i].D)

    thisdict = dict(volume = bestVolume, score = bestSoFar, maxD = maxD)


    # ACO phase
    bestIter = -1
    iter = 0
    stagnant = 0
    iterPallets = [] # the number of iterations is not known
    iterTorque  = []
    iterItems   = []

    while stagnant < 3 and (time.perf_counter() - startTime) < secBreak: # iterations

        bestLocalScore  = 0
        bestLocalVolume = 0

        if mode == "Serial":

            iterPallets.append( copy.deepcopy(pallets) )
            iterTorque.append( solTorque )
            iterItems.append( solItems )
           
            antSolve(iterPallets[iter], items, cfg, k, secBreak, iterTorque[iter], iterItems[iter], Attract, Phero,\
                 lock, bestSoFar, stat, thisdict)

            bestLocalScore    = thisdict["score"]
            bestLocalVolume   = thisdict["volume"]             

        else: # parallel

            ants       = [None for _ in np.arange(NANTS)]
            dicts      = [None for _ in np.arange(NANTS)] # for ant scores and volumes
            antPallets = [None for _ in np.arange(NANTS)]
            antTorque  = [None for _ in np.arange(NANTS)]
            antItems   = [None for _ in np.arange(NANTS)]
            dicts      = [None for _ in np.arange(NANTS)]

            bestAnt = -1

            # parallel shims phase
            for i, _ in enumerate(ants): # ants

                antPallets[i] = copy.deepcopy(pallets)
                antTorque[i]  = solTorque
                antItems[i]   = solItems
                dicts[i]      = thisdict # for ant scores and volumes

                ants[i] = mp.Process( target=antSolve, args=( antPallets[i], items, cfg, k, secBreak,\
                    antTorque[i], antItems[i], Attract, Phero, lock, bestSoFar, stat, dicts[i]) )

                ants[i].start()
                    
            while time.perf_counter() - startTime <= secBreak:
                if not any(p.is_alive() for p in ants):
                    print(f"All the ants of iteration {iter} have ended.")
                    break
            else:
                # We only enter this if we didn't 'break' above.
                print("timed out, killing all ants")
                for p in ants:
                    p.terminate()
                    p.join()

            iterPallets.append( None )
            iterTorque.append( None )
            iterItems.append( None ) 

            for i, _ in enumerate(ants):

                if dicts[i]["score"] > bestLocalScore or dicts[i]["volume"] > bestLocalVolume:
                    bestLocalScore  = dicts[i]["score"] 
                    bestLocalVolume = dicts[i]["volume"] 
                    bestAnt = i
                    iterPallets[iter] = copy.deepcopy(antPallets[i])
                    iterTorque[iter]  = antTorque[i]                    
                    iterItems[iter]   = antItems[i]

        if bestLocalScore > bestSoFar or bestLocalVolume > bestVolume:
            bestSoFar  = bestLocalScore
            bestVolume = bestLocalVolume
            bestIter    = iter
            pallets    = copy.deepcopy(iterPallets[iter])
            solTorque  = iterTorque[iter]
            solItems   = iterItems[iter]
            stagnant = 0
        else:
            stagnant += 1

        iter += 1

    print(f"Best iteration {bestIter}/{iter}.")
    if mode == "Parallel":
        print(f"Best ant {bestAnt}/{NANTS}.")

        
if __name__ == "__main__":

    print("----- Please execute module main -----")
