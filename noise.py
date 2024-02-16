
import common
import time
import math
import numpy as np
import multiprocessing as mp
import common
import icecream as ic

RNG = np.random.default_rng()

def Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, iterTorque, k, cfg, lock, startTime, nodeTime):

    # make an elementary feasible transformation in the solution
    transformed = False

    i = RNG.integers(len(iterPallets))

    notIncluded  = [j for j, a in enumerate(iterSolDict["solMatrix"][N:N+(N-1)])     if a == 0] # not included items
    inThisPallet = [j for j, a in enumerate(iterSolDict["solMatrix"][N*i:N*i+(N-1)]) if a == 1] # in this pallet

    while not transformed:

        id0 = RNG.choice(notIncluded)
        notIncluded.remove(id0)

        if len(inThisPallet) > 0:
            id1 = RNG.choice(inThisPallet)
            inThisPallet.remove(id1)
        else:
            break

        # remove item id1 from this pallet
        iterPallets[i].popItem(items[id1], iterTorque, iterSolDict, N, iterItemsDict)
        
        # try to include id0
        if iterPallets[i].isFeasible(items[id0], 1.0, k, iterTorque, cfg, iterItemsDict, lock):
            iterPallets[i].putItem(items[id0], iterTorque, iterSolDict, N, iterItemsDict, lock)
            transformed = True
        else:
            # put id1 back into this pallet
            iterPallets[i].putItem(items[id1], iterTorque, iterSolDict, N, iterItemsDict, lock)

    iterScore += iterPallets[i].PCS


# ProbAccept is the Noising Method acceptance method.
# This feature prevents the method from becoming stuck at a local optima.
def ProbAccept(newScore, oldScore, r):

    ratio = float(newScore-oldScore) / float(oldScore)

    delta = ratio + r*(2*RNG.random() - 1.0)

    return delta > 0 # may return true or false


def Solve(pallets, items, cfg, k, nodeTime, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items)
    M = len(pallets)

    print("\nNoising Method Optimization for ACLP+RPDP\n")        

    lock  = mp.Lock() # for use in parallel mode

    print(f"{N} items  {M} pallets")

    initPallets   = common.copyPallets(pallets)
    initSolDict   = dict(solDict)
    initItemsDict = dict(itemsDict)
    initTorque    = mp.Value('d', nodeTorque.value)  
    initScore     = 0.0
    for i, _ in enumerate(initPallets):               
        common.fillPallet(initPallets[i], items, k, initTorque, initSolDict, cfg, 1.0, initItemsDict, lock)
        initScore += initPallets[i].PCS

    bestScore = initScore # G*

    primePallets   = common.copyPallets(pallets)
    primeSolDict   = dict(solDict)
    primeItemsDict = dict(itemsDict)
    primeTorque    = mp.Value('d', nodeTorque.value)  
    primeScore     = 0.0
    for i, _ in enumerate(primePallets):               
        common.fillPallet(primePallets[i], items, k, primeTorque, primeSolDict, cfg, 0.95, primeItemsDict, lock)
        primeScore += primePallets[i].PCS

    r_max = 1 - (initScore-primeScore)/initScore # maximum initial noise

    print(f"r_max:{r_max:.3f}, initScore:{initScore}, primeScore:{primeScore}")

    numTrials = math.ceil(float(N * M)/100)

    numIter = math.ceil(float(N * M)/50)

    print(f"numTrials: {numTrials}\tnumIter: {numIter}")

    step = r_max/(numTrials-1)

    print(f"step = {step:.6f}")

    r = r_max  
  
    trial = 0
    while trial < numTrials and (time.perf_counter() - startTime) < nodeTime:

        localScore     = initScore
        localSolDict   = dict(solDict)
        localItemsDict = dict(itemsDict)
        localTorque    = mp.Value('d', nodeTorque.value)
        localPallets    = common.copyPallets(pallets)

        iterScore      = initScore
        iterSolDict    = dict(solDict)
        iterItemsDict  = dict(itemsDict)
        iterTorque     = mp.Value('d', nodeTorque.value)
        iterPallets    = common.copyPallets(pallets)

        for _ in range(numIter):


            if ((time.perf_counter() - startTime) > nodeTime):
                trial = numTrials
                break 

            iterScore = localScore

            oldScore = iterScore

            # Transform makes a random elementary transformation in each pallet
            Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, iterTorque, k, cfg, lock, startTime, nodeTime)

            if ProbAccept(iterScore, oldScore, r):

                localScore     = iterScore
                localSolDict   = dict(iterSolDict)
                localItemsDict = dict(iterItemsDict)
                localPallets   = common.copyPallets(iterPallets)
                localTorque    = mp.Value('d', iterTorque.value)

        r -= step

        if localScore > bestScore:
            bestScore = localScore
            solDict    = dict(localSolDict)
            itemsDict  = dict(localItemsDict)
            pallets    = common.copyPallets(localPallets)
            nodeTorque = mp.Value('d', localTorque.value)

        trial += 1


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")