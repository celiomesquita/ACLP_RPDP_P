
import common
import time
import math
import numpy as np
import multiprocessing as mp
import common
# import random

RNG = np.random.default_rng()

def Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, nodeTorque, k, cfg, lock):

    for i, _ in enumerate(iterPallets):

        arr0 = [j for j, a in enumerate(iterItemsDict["mpItems"])                if a == 0]
        arr1 = [j for j, a in enumerate(iterSolDict["solMatrix"][N*i:N*i+(N-1)]) if a == 1]

        if len(arr0) > 0 and len(arr1) > 0:
            # id0 = np.random.choice(arr0) # not in solution
            # id1 = np.random.choice(arr1) # in this pallet

            id0 = RNG.choice(arr0) # not in solution
            id1 = RNG.choice(arr1) # in this pallet            

            iterPallets[i].popItem(items[id1], nodeTorque, iterSolDict, N, iterItemsDict)
            
            if iterPallets[i].isFeasible(items[id0], 1.0, k, nodeTorque, cfg, iterItemsDict, lock, torqueSurplus=1.0):
                iterPallets[i].putItem(items[id0], nodeTorque, iterSolDict, N, iterItemsDict, lock)
            else:
                iterPallets[i].putItem(items[id1], nodeTorque, iterSolDict, N, iterItemsDict, lock)


        iterScore += iterPallets[i].PCS



# ProbAccept is the Noising Method acceptance method.
# This feature prevents the method from becoming stuck at a local optima.
def ProbAccept(newScore, oldScore, r):
	delta = float(newScore-oldScore) / float(oldScore)
	return delta + (2*r*RNG.random() - r)  > 0 


def Solve(pallets, items, cfg, k, secBreak, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items)
    M = len(pallets)

    print("\nNoising Method Optimization for ACLP+RPDP\n")        

    lock  = mp.Lock() # for use in parallel mode

    print(f"{N} items  {M} pallets")

    initScore = 0.0 # G

    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock)
        initScore += pallets[i].PCS

    print(f"Greedy initial score {initScore}")

    bestScore = initScore # G*

    primePallets   = common.copyPallets(pallets)
    primeSolDict   = common.copySolDict(solDict)
    primeItemsDict = common.copyItemsDict(itemsDict)
    primeTorque    = mp.Value('d', nodeTorque.value)  
    primeScore = 0.0
    for i, _ in enumerate(primePallets):               
        common.fillPallet(primePallets[i], items, k, primeTorque, primeSolDict, cfg, 0.7, primeItemsDict, lock)
        primeScore += primePallets[i].PCS

    r_max = 1 - (initScore - primeScore)/initScore # maximum initial noise

    # initialize the edges between pallets and items
    # Ek = [None] * N * M
    # id = 0
    # for p in pallets:
    #     for it in items:
    #         Ek[id] = common.Edge(id, p, it, cfg)
    #         id += 1  

    numTrials = math.ceil(N * M / 50)


    numIter = int(numTrials/2)

    step = r_max/(numTrials/numIter-1)
    r = r_max  

    # the most attractive edges come first
    # Ek.sort(key=lambda  x: x.Attract, reverse=True)

    bestScore = initScore

    trial = 0
    while trial < numTrials and (time.perf_counter() - startTime) < secBreak:

        localScore     = initScore
        localSolDict   = common.copySolDict(solDict)
        localItemsDict = common.copyItemsDict(itemsDict)
        localTorque    = mp.Value('d', nodeTorque.value)
        localPallets    = common.copyPallets(pallets)

        iterScore      = initScore
        iterSolDict    = common.copySolDict(solDict)
        iterItemsDict  = common.copyItemsDict(itemsDict)
        iterTorque     = mp.Value('d', nodeTorque.value)
        iterPallets    = common.copyPallets(pallets)

        for _ in range(numIter):


            if ((time.perf_counter() - startTime) > secBreak):
                trial = numTrials
                break 

            iterScore = localScore

            oldScore = iterScore

            # Transform makes a random elementary transformation in each pallet
            Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, iterTorque, k, cfg, lock)

            if ProbAccept(iterScore, oldScore, r):

                localScore     = iterScore
                localSolDict   = common.copySolDict(iterSolDict)
                localItemsDict = common.copyItemsDict(iterItemsDict)
                localPallets    = common.copyPallets(iterPallets)
                localTorque    = mp.Value('d', iterTorque.value)

        r -= step

        if localScore > bestScore:

            bestScore = localScore

            solDict    = common.copySolDict(localSolDict)
            itemsDict  = common.copyItemsDict(localItemsDict)
            pallets    = common.copyPallets(localPallets)
            nodeTorque = mp.Value('d', localTorque.value)

        trial += 1


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")