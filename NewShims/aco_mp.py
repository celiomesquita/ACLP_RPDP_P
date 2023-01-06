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

def antSolve(antPallets, items, cfg, k, secBreak, antTorque, antItems, Attract, Phero, lock, bestScore,\
     stat, accum, maxD, startTime):

    # items are not changed
    # antItems are changed by this ant

    nbhood   = [it for it in items]
    values   = [v  for v  in Attract]


    # accum["score"]  -> accumulated score
    # accum["volume"] -> accumulated volume

    while nbhood and (time.perf_counter() - startTime) < secBreak:

        for i, p in enumerate(antPallets):
            
            if len(nbhood) > 0:

                maxTorque = max([it.W * maxD for it in nbhood])

                # pick from the neighborhood the probable best item for this pallet
                item, j = selectItem(nbhood, values, p, maxTorque)

                if antPallets[i].isFeasible(item, 1.0, k, antTorque, antItems, lock, cfg ):

                    antPallets[i].putItem(item, antTorque, antItems, lock)

                    accum["score"]  += item.S
                    accum["volume"] += item.V

                    nbhood.pop(j) # pop if included in solution
                    values.pop(j)

                else: # pop in the last pallet tested
                    if i == len(antPallets)-1:
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

    # if accum["volume"] > 1.0:
    #     accum["volume"] = 1.0

def copyPallets(pallets):
    array = [None for _ in pallets]
    for i, p in enumerate(pallets):
        array[i] = common.Pallet(p.ID, p.D, p.V, p.W, 1)
        array[i].Dests = p.Dests
        array[i].PCW   = p.PCW 
        array[i].PCV   = p.PCV
        array[i].PCS   = p.PCS
    return array

def copySolItems(solItems):
    mp_array = mp.Array('i', range(len(solItems)))
    for j, v in enumerate(solItems):
        mp_array[j] = v
    return mp_array

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


    # greedy serial phase (both for serial and parallel modes)---------------------

    initScore   = 0.0 # best score so far
    initVolume  = 0.0 # best volume so far
    maxD        = 0.0 # the maximum distance from the CG

    pallets.sort(key=lambda x: abs(x.D)) # sort pallets ascendent by CG distance

    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, solTorque, solItems, lock, cfg, limit)
        initScore  += pallets[i].PCS
        initVolume += pallets[i].PCV   
        if abs(pallets[i].D) > maxD:
            maxD = abs(pallets[i].D)

    bestScore  = initScore  # best score so far
    bestVolume = initVolume # best volume so far

    # ACO phase -------------------------------------------------------------------
    iter     = 0
    stagnant = 0
    bi       = 0 # best iteration

    numAnts  = 1 # number of ants per iteration
    if mode == "Parallel":
        numAnts = 8

    iterPallets = []
    iterTorque  = []
    iterItems   = []

    while stagnant < 5: # iterations 

        # modified parameters from the greedy phase
        iterPallets.append( pallets )
        iterTorque.append( solTorque )
        iterItems.append( solItems ) 

        # initialize ants parameters
        ants       = [None for _ in np.arange(numAnts)]
        accums     = [None for _ in np.arange(numAnts)] # for ant scores and volumes
        antPallets = [None for _ in np.arange(numAnts)]
        antTorque  = [None for _ in np.arange(numAnts)]
        antItems   = [None for _ in np.arange(numAnts)]

        bestAntScore  = initScore
        bestAntVolume = initVolume

        # parallel phase
        for a, _ in enumerate(ants): # ants

            antPallets[a] =  copyPallets(iterPallets[iter])              
            antTorque[a]  = iterTorque[iter]
            antItems[a]   = copySolItems(iterItems[iter] )
            accums[a]     = dict(score=initScore, volume=initVolume)

            ants[a] = mp.Process( target=antSolve, args=( antPallets[a], items, cfg, k, secBreak,\
                antTorque[a], antItems[a], Attract, Phero, lock, bestScore, stat, accums[a], maxD, startTime) )

            ants[a].start() # send ant
        
        # wait until time limit or all ants finish their jobs
        while time.perf_counter() - startTime <= secBreak:
            if not any(ant.is_alive() for ant in ants): # if all the ants have ended.
                break
        else:
            # if time is over, all ants are freed
            for ant in ants:
                ant.terminate()

        # look for the best ant solution
        ba = 0 # best ant index
        for a, ant in enumerate(ants):
            ant.join() 
            if accums[a]["score"] > bestAntScore or accums[a]["volume"] > bestAntVolume :
                bestAntScore  = accums[a]["score"] 
                bestAntVolume = accums[a]["volume"] 
                ba = a
                print("a better solution found by an ant!")   

        # iteration solution update
        if bestAntScore > bestScore or bestAntVolume > bestVolume:
            bestScore  = bestAntScore
            bestVolume = bestAntVolume
            iterPallets[iter] = copyPallets(antPallets[ba]) 
            iterTorque[iter]  = antTorque[ba] 
            iterItems[iter]   = copySolItems(antItems[ba])
            bi = iter
            stagnant = 0
            print("a better solution found by an iteration!") 
        else:
            stagnant += 1

        iter += 1   

    pallets    = copy.deepcopy(iterPallets[bi])
    solTorque  = iterTorque[bi] 
    solItems   = iterItems[bi]   

    print(f"Best ant {ba}/{numAnts}.\t{bi}/{iter} Iterations. ({numAnts*iter} total ants)\n")

        
if __name__ == "__main__":

    print("----- Please execute module main -----")
