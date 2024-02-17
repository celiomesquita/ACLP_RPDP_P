
import common
import time
import math
import numpy as np
import multiprocessing as mp
import common
import icecream as ic
import tqdm

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
                iterScore -= items[id1].S
                
                # try to include id0
                if iterPallets[i].isFeasible(items[id0], 1.0, k, iterTorque, cfg, iterItemsDict, lock):
                    iterPallets[i].putItem(items[id0], iterTorque, iterSolDict, N, iterItemsDict, lock)
                    transformed = True
                    iterScore += items[id0].S
                else:
                    # put id1 back into this pallet
                    iterPallets[i].putItem(items[id1], iterTorque, iterSolDict, N, iterItemsDict, lock)
                    iterScore += items[id1].S

        if len(tested) == len(iterPallets):
            return iterScore
        
    return iterScore


# ProbAccept is the Noising Method acceptance method.
# This feature prevents the method from becoming stuck at a trial optima.
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

    bestScore = float(initScore) # G*

    r_init = 0.3

    numTrials = math.ceil(float(N))

    numIters = int(numTrials/1)

    step = r_init/(numTrials-1)


    r = r_init 

    # initialize the trial solution
    trialScore     = float(initScore)
    trialSolDict   = dict(solDict)
    trialItemsDict = dict(itemsDict)
    trialTorque    = mp.Value('d', nodeTorque.value)
    trialPallets    = common.copyPallets(pallets)    
    """"""
    trial = 0
    while trial < numTrials:

        # update the trial solution (back and forth)

        for _ in range(numIters):

            if ((time.perf_counter() - startTime) > nodeTime):
                trial = numTrials
                break

            iterScore      = float(trialScore)
            iterSolDict    = dict(trialSolDict)
            iterItemsDict  = dict(trialItemsDict)
            iterTorque     = mp.Value('d', trialTorque.value)
            iterPallets    = common.copyPallets(trialPallets)

            # Transform makes a random elementary transformation in a randomly chosen pallet
            iterScore = Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, N, items, iterTorque, k, cfg, lock)

            if ProbAccept(iterScore, trialScore, r):
            # if iterScore > trialScore:
                trialScore     = float(iterScore)
                trialSolDict   = dict(iterSolDict)
                trialItemsDict = dict(iterItemsDict)
                trialPallets   = common.copyPallets(iterPallets)
                trialTorque    = mp.Value('d', iterTorque.value)

        r -= step

        if trialScore > bestScore:
            bestScore  = float(trialScore)
            solDict    = dict(trialSolDict)
            itemsDict  = dict(trialItemsDict)
            pallets    = common.copyPallets(trialPallets)
            nodeTorque = mp.Value('d', trialTorque.value)
            break # first improvement

        trial += 1
    """"""
    print(f"trials:{numTrials}\titers:{numIters}\tstep:{step:.5f}\t ratio:{trialScore/bestScore:.5f}")


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")