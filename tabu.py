
import common
import time
import math
import numpy as np
import common
import multiprocessing as mp
import random

RNG = np.random.default_rng()

def Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, iterSolID, N, items, iterTorque, k, cfg, lock):

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
        
        iterSolID.value += int(iterPallets[i].PCS + iterPallets[i].PCW + iterPallets[i].PCV)


def Solve(pallets, items, cfg, pi, k, secBreak, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items)
    M = len(pallets)

    # 2% the problem size
    tq_size  = math.ceil( float(N*M) / 50 )
    numIters = 10*tq_size

    tq = []

    # the bigger the problem less iterations (244 - 27 iterations)
    # numIter = math.ceil( 600_000.0 / float(len_tq) )

    print(f"\nTabu Search for ACLP+RPDP ({pi}-{k})")        
    print(f"{len(items)} items  {len(pallets)} pallets")

    lock  = mp.Lock() # for use in parallel mode

    initScore = mp.Value('d', 0) # G

    initPallets   = common.copyPallets(pallets)
    initSolDict   = dict(solDict)
    initItemsDict = dict(itemsDict)
    initTorque    = mp.Value('d', nodeTorque.value) 
    initScore     = mp.Value('d', 0)
    initSolID     = mp.Value('d', 0)

    for i, _ in enumerate(initPallets):               
        common.fillPallet(initPallets[i], items, k, initTorque, initSolDict, cfg, 1.0, initItemsDict, lock)
        initScore.value += initPallets[i].PCS
        initSolID.value += int(pallets[i].PCS + pallets[i].PCW + pallets[i].PCV)

    bestScore  = mp.Value('d', initScore.value)
    bestSolID  = mp.Value('d', initSolID.value)

    # initialize the trial solution
    trialPallets   = common.copyPallets(initPallets)  
    trialSolDict   = dict(initSolDict)
    trialItemsDict = dict(initItemsDict)
    trialTorque    = mp.Value('d', initTorque.value)
    trialScore     = mp.Value('d', initScore.value)
    trialSolID     = mp.Value('d', initSolID.value)

    iterTorque = mp.Value('d', initTorque.value)
    iterScore  = mp.Value('d', initTorque.value)
    iterSolID  = mp.Value('d', initSolID.value)

    # TS phase -------------------------------------------------------------------

    smax = 3

    stagnant = 0
    while stagnant < smax:

        for iter in range(numIters): # local solution iterations

            if ((time.perf_counter() - startTime) > secBreak):
                stagnant = smax
                break            

            iterPallets      = common.copyPallets(trialPallets)
            iterSolDict      = dict(trialSolDict)
            iterItemsDict    = dict(trialItemsDict)
            iterTorque.value = trialTorque.value
            iterScore.value  = trialScore.value
            iterSolID.value  = trialSolID.value

            # Transform makes a random elementary transformation in each pallet
            Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, iterSolID, N, items, nodeTorque, k, cfg, lock)

            if iterScore.value > trialScore.value+0.001 and iterSolID not in tq:

                trialScore     = iterScore
                trialSolDict   = dict(iterSolDict)
                trialItemsDict = dict(iterItemsDict)
                trialPallets   = common.copyPallets(iterPallets)
                trialSolID     = iterSolID            

        if trialSolID not in tq:
            tq.append(trialSolID)
        
        if len(tq) > tq_size: # pop the first oldest solution
            tq.pop(0)   

        if trialScore.value  > bestScore.value + 0.001 :
            pallets    = common.copyPallets(trialPallets)
            solDict    = dict(trialSolDict)
            itemsDict  = dict(trialItemsDict)
            nodeTorque.value = trialTorque.value
            bestScore.value  = trialScore.value
            bestSolID.value  = trialScore.value
            stagnant = 0
        else:
            stagnant += 1
    """"""
    print(f"TQ size:{tq_size}   iters:{numIters}   ratio:{(bestScore.value-initScore.value)/initScore.value:.3f}")
 
if __name__ == "__main__":

    print("----- Please execute the main py file -------------")