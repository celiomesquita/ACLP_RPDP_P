
import common
import time
import math
import numpy as np
import multiprocessing as mp
import common
import icecream as ic
import tqdm
import os

RNG = np.random.default_rng()

def Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, iterTorque, k, cfg, lock):

    # make an elementary feasible transformation in the solution
    transformed = False

    tested = []

    while not transformed:

        i = RNG.integers(len(iterPallets))
        
        if i not in tested:
            
            tested.append(i)

            notIncluded  = [j for j, a in enumerate(iterItemsDict["mpItems"])                if a == 0] # not included items
            inThisPallet = [j for j, a in enumerate(iterSolDict["solMatrix"][N*i:N*i+(N-1)]) if a == 1] # in this pallet

            while not transformed and len(notIncluded) > 0:

                id0 = RNG.choice(notIncluded)
                notIncluded.remove(id0)

                if len(inThisPallet) > 0:
                    id1 = RNG.choice(inThisPallet)
                    inThisPallet.remove(id1)
                else:
                    break

                # remove item id1 from this pallet
                iterPallets[i].popItem(items[id1], iterTorque, iterSolDict, N, iterItemsDict)
                iterScore.value -= items[id1].S
                
                # try to include id0
                if iterPallets[i].isFeasible(items[id0], 1.0, k, iterTorque, cfg, iterItemsDict, lock):
                    iterPallets[i].putItem(items[id0], iterTorque, iterSolDict, N, iterItemsDict, lock)
                    transformed = True
                    iterScore.value += items[id0].S
                else:
                    # put id1 back into this pallet
                    iterPallets[i].putItem(items[id1], iterTorque, iterSolDict, N, iterItemsDict, lock)
                    iterScore.value += items[id1].S

        if len(tested) == len(iterPallets):
            return iterScore
        
    return iterScore

def writeDelta(value):

    dirname = f"./results/NMO/"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass  

    fname = f"{dirname}/NMO_delta.res"    
        
    line = f"{value}\n"

    writer = open(fname, "a+") 
    try:
        writer.write(line)
    finally:
        writer.close() 

# ProbAccept is the Noising Method acceptance method.
# This feature prevents the method from becoming stuck at a trial optima.
def ProbAccept(newScore, oldScore, r):

    # ratio = (newScore.value-oldScore.value) / oldScore.value # may be positive or negative
    ratio = (newScore.value-oldScore.value)

    u = 2.*RNG.random()-1. # -1 to 1

    # if delta is positive and bigger  than +ru it returns True
    # if delta is negative and smaller than -ru it returns False
    # if |delta| is smaller than |r * u| it may return True or False
    delta = ratio + r * u

    # writeDelta(f"{delta:.5f}")

    return delta > 0 # may return true or false

def Solve(pallets, items, cfg, pi, k, nodeTime, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items)
    M = len(pallets)

    print(f"\nNoising Method Optimization for ACLP+RPDP ({pi}-{k})")        
    print(f"{N} items  {M} pallets")

    lock  = mp.Lock() # for use in parallel mode

    initPallets   = common.copyPallets(pallets)
    initSolDict   = dict(solDict)
    initItemsDict = dict(itemsDict)
    initTorque    = mp.Value('d', nodeTorque.value) 
    initScore     = mp.Value('d', 0)

    for i, _ in enumerate(initPallets):               
        common.fillPallet(initPallets[i], items, k, initTorque, initSolDict, cfg, 1.0, initItemsDict, lock)
        initScore.value += initPallets[i].PCS

    # r_init = 0.002 # must be smaller than the ratio 0.004 - 0.008
    r_init = 35.

    numIters = int(N/5) # number of elementary transformations in a trial solution
    numTrials = int(numIters/5)

    # numTrials = math.ceil( float(N * M) / 10 ) # too long
    # numIters = int(numTrials/2)

    step = r_init/(numTrials-1)

    r = r_init 

    # initialize the trial solution
    bestPallets   = common.copyPallets(initPallets)  
    bestSolDict   = dict(initSolDict)
    bestItemsDict = dict(initItemsDict)
    bestTorque    = mp.Value('d', initTorque.value)
    bestScore     = mp.Value('d', initScore.value)

    trialTorque    = mp.Value('d', bestTorque.value)
    trialScore     = mp.Value('d', bestScore.value)
    iterTorque = mp.Value('d', initTorque.value)
    iterScore  = mp.Value('d', initTorque.value)

    """"""

    maxNoise = 0.0
    trial = 0
    while trial < numTrials:

        # initialize the trial solution
        trialPallets   = common.copyPallets(bestPallets)  
        trialSolDict   = dict(bestSolDict)
        trialItemsDict = dict(bestItemsDict)
        trialTorque.value  = bestTorque.value
        trialScore.value   = bestScore.value

        # update the trial solution (back and forth)

        for _ in range(numIters):

            if ((time.perf_counter() - startTime) > nodeTime):
                trial = numTrials
                break

            iterPallets      = common.copyPallets(trialPallets)
            iterSolDict      = dict(trialSolDict)
            iterItemsDict    = dict(trialItemsDict)
            iterTorque.value = trialTorque.value
            iterScore.value  = trialScore.value

            # Transform makes a random elementary transformation in a randomly chosen pallet
            Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, iterTorque, k, cfg, lock)


            # noise = abs(iterScore.value - trialScore.value)/trialScore.value
            noise = abs(iterScore.value - trialScore.value)
            if noise > maxNoise:
                maxNoise = noise

            if ProbAccept(iterScore, trialScore, r):
            # if iterScore.value > trialScore.value + 0.0001:
                trialPallets       = common.copyPallets(iterPallets)
                trialSolDict       = dict(iterSolDict)
                trialItemsDict     = dict(iterItemsDict)
                trialTorque.value  = iterTorque.value
                trialScore.value   = iterScore.value 

        r -= step

        if trialScore.value > bestScore.value + 0.001 :
            bestPallets    = common.copyPallets(trialPallets)
            bestSolDict    = dict(trialSolDict)
            bestItemsDict  = dict(trialItemsDict)
            bestTorque.value = trialTorque.value
            bestScore.value  = trialScore.value
            # break # first improvement

        trial += 1

    pallets    = common.copyPallets(bestPallets)  
    solDict    = dict(bestSolDict)
    itemsDict  = dict(bestItemsDict)
    nodeTorque.value = bestTorque.value
    bestScore.value  = bestScore.value

    """"""
    print(f"trials:{numTrials}   iters:{numIters}   step:{step:.5f}   ratio:{(bestScore.value-initScore.value)/initScore.value:.3f}   maxNoise:{maxNoise:.3f}")


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")