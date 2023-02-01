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
BETA  = 3 # heuristic exponent

# Ant System (AS), the classic method that uses a random proportional state transition rule,
# while the pheromone is deposited by all ants proportionally to their solution quality and
# is evaporated in all the components. Each Ant makes use of "depositPhero" to update 
# item attractiveness according to the its solution value.
def depositPhero(score, bestScore, Attract, Phero, items):

    deltaTau = (score - bestScore)/bestScore

    if deltaTau > 0: # if improved...

        for id, _ in enumerate(Phero):

            Phero[id] += deltaTau

            Attract[id] = Phero[id]**ALPHA * items[id].Attr**BETA

def evaporate(Attract, Phero, items):

        for id, phero in enumerate(Phero):

            #evaporate some pheromone 
            Phero[id] = math.sqrt(phero) / 1.35

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

def antSolve(antPallets, items, cfg, k, secBreak, antTorque, antSolDict, Attract, Phero,\
     bestScore, maxD, startTime, accumsP, accum, itemsDict, lock, mp=False):

    # items are read only
    # antSolDict are changed by this ant

    nbhood   = [it for it in items]
    values   = [v  for v  in Attract]
    N = len(items)

    # accum['score'] -> accumulated score for serial mode
    # accumsP.value  -> accumulated score for parallel mode

    while nbhood and (time.perf_counter() - startTime) < secBreak:

        for i, p in enumerate(antPallets):
            
            if len(nbhood) > 0:

                maxTorque = max([it.W * maxD for it in nbhood])

                # pick from the neighborhood the probable best item for this pallet
                item, j = selectItem(nbhood, values, p, maxTorque)

                if antPallets[i].isFeasible(item, 1.0, k, antTorque, cfg, itemsDict, lock):

                    antPallets[i].putItem(item, antTorque, antSolDict, N, itemsDict, lock)

                    if mp:
                        with lock:
                            accumsP.value  += item.S
                            Phero[item.ID] -= item.S/bestScore # dissimulate other parallel ants
                            Attract[item.ID] = Phero[item.ID]**ALPHA * items[item.ID].Attr**BETA                    

                    else: 
                        accum["score"]  += item.S
                        Phero[item.ID] -= item.S/bestScore # dissimulate the following ants
                        Attract[item.ID] = Phero[item.ID]**ALPHA * items[item.ID].Attr**BETA  

                    nbhood.pop(j) # pop if included in solution
                    values.pop(j)

                else: # pop in the last pallet tested
                    if i == len(antPallets)-1:
                        nbhood.pop(j)
                        values.pop(j)        


def Solve( pallets, items, cfg, k, limit, secBreak, mode, solTorque, solDict, itemsDict ):

    # to control the general attractiveness for the tournament selection
    Attract = mp.Array('d', np.arange(len(items)))
    for j, _ in enumerate(Attract):
        Attract[j] = 0.5

    # to control pheromone deposition and evaporation
    Phero = mp.Array('d', np.arange(len(items)))
    for j, _ in enumerate(Phero):
        Phero[j] = 0.5

    startTime = time.perf_counter()

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    print(f"\n{mode} Ant Colony Optimization for ACLP+RPDP\n")        

    lock  = mp.Lock() # for use in parallel mode

    # greedy serial phase (both for serial and parallel modes)---------------------

    maxD      = 0.0 # the maximum distance from the CG
    bestScore = 0.0
    pallets.sort(key=lambda x: abs(x.D)) # sort pallets ascendent by CG distance

    print(f"{len(items)} items  {len(pallets)} pallets")

    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, solTorque, solDict, cfg, limit, itemsDict, lock)
        bestScore  += pallets[i].PCS
        if abs(pallets[i].D) > maxD:
            maxD = abs(pallets[i].D)
    
    initPallets   = common.copyPallets(pallets)              
    initTorque    = solTorque
    initSolDict   = common.copySolDict(solDict) 
    initItemsDict = common.copyItemsDict(itemsDict)
    initScore    = bestScore

    print(f"Greedy initial score {initScore}")

    # ACO phase -------------------------------------------------------------------
    iter     = 0
    stagnant = 0
    numAnts  = 8 # number of ants per iteration
    improvements = 0
    bi = 0 # best iter

    iterPallets = []
    iterTorque  = []
    iterSolDict = []
    iterItemsDict = []
    bestIterScore = initScore

    while stagnant < 5: # iterations

        iterPallets.append(None)
        iterTorque.append(None)
        iterSolDict.append(None)
        iterItemsDict.append(None)        

        # initialize ants parameters
        ants       = [None          for _ in np.arange(numAnts)]
        accumsS    = [None          for _ in np.arange(numAnts)] # for ant scores and volumes in serial mode
        accumsP    = [mp.Value('d') for _ in np.arange(numAnts)] # for ant scores and volumes in parallel mode
        antPallets = [None          for _ in np.arange(numAnts)]
        antTorque  = [None          for _ in np.arange(numAnts)]
        antSolDict = [None          for _ in np.arange(numAnts)]
        antItemsDict = [None        for _ in np.arange(numAnts)]

        bestAntScore = initScore

        # stats = dict( AttractVar = 0.0, PheroVar = 0.0, AttractMean = 0.0, PheroMean = 0.0)                 
      
        # parallel phase
        ba = 0 # best ant
        for a, _ in enumerate(ants): # ants

            # modified parameters from the greedy phase
            antPallets[a]   = common.copyPallets(initPallets)              
            antTorque[a]    = initTorque
            antSolDict[a]   = common.copySolDict(initSolDict)
            antItemsDict[a] = common.copyItemsDict(initItemsDict)

            # initialize accumulated scores
            accumsS[a]       = dict(score=initScore) # for serial   mode
            accumsP[a].value = 0.0                   # for parallel mode

            if mode == "Parallel": # send "antSolve" to parallel processes (ants)

                ants[a] = mp.Process( target=antSolve, args=( antPallets[a], items, cfg, k, secBreak,\
                    antTorque[a], antSolDict[a], Attract, Phero, bestScore, maxD, startTime, accumsP[a],\
                         accumsS[a], antItemsDict[a], lock, True) )

                ants[a].start() # send ant

            else: # solve sequentially

                antSolve( antPallets[a], items, cfg, k, secBreak, antTorque[a], antSolDict[a], Attract, Phero,\
                    bestScore, maxD, startTime, accumsP[a], accumsS[a], antItemsDict[a], lock)

                # Ant System: all ants deposit pheromone
                depositPhero(accumsS[a]['score'], bestScore, Attract, Phero, items)

                # serial ant best solution update  accumsS[a]['score']
                if accumsS[a]['score'] > bestAntScore:
                    bestAntScore  = accumsS[a]['score'] 
                    ba = a
                    improvements += 1


        if mode == "Parallel":
            # wait until time limit or all ants finish their jobs
            while time.perf_counter() - startTime <= secBreak:
                if not any(ant.is_alive() for ant in ants): # if all the ants have ended.
                    break
            else:
                # if time is over, all ants are freed
                for ant in ants:
                    ant.terminate()

            # look for the best ant solution
            for a, _ in enumerate(ants):
                ants[a].join() # get results from parallel ants

                # Ant System: all ants deposit pheromone
                depositPhero(accumsP[a].value, bestScore, Attract, Phero, items)

                # parallel ant best solution update
                if accumsP[a].value > bestAntScore:
                    bestAntScore  = accumsP[a].value 
                    ba = a
                    improvements += 1


        # iteration best solution update
        if bestAntScore > bestIterScore:
            bestIterScore = bestAntScore
            print(f"Best iter score {bestIterScore} ({iter})")
            iterSolDict[iter] = common.copySolDict( antSolDict[ba] ) 
            iterItemsDict[iter] = common.copyItemsDict( antItemsDict[ba] )
            bi = iter
            stagnant = 0
        else:
            stagnant += 1


        iter += 1

        evaporate(Attract, Phero, items)
  
    print(f"{improvements} improvements ({numAnts*iter} total ants).")

    if iterSolDict[bi] != None:
        solDict   = common.copySolDict( iterSolDict[bi] )
        itemsDict = common.copyItemsDict( iterItemsDict[bi] )
                
    # N = len(items)
    # Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)
    # counter = 0
    # for i, row in enumerate(Y):
    #     for j, X_ij in enumerate(row):
    #         if X_ij == 0 and pallets[i].isFeasible(items[j], limit, k, solTorque, solDict, lock, cfg, N, itemsDict):
    #             pallets[i].putItem( items[j], solTorque, solDict, lock, N, itemsDict)
    #             counter += 1
    # print(f"---> {counter} items inserted by the local search.") 

    AttractVar  = statistics.variance(Attract)
    PheroVar    = statistics.variance(Phero)
    AttractMean = statistics.mean(Attract)
    PheroMean   = statistics.mean(Phero)
    print(f"{AttractVar:.1f}\t{AttractMean:.1f}\t{PheroVar:.3f}\t{PheroMean:.3f}")
        


if __name__ == "__main__":

    print("----- Please execute module main -----")
