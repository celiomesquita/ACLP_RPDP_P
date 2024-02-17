
import common
import time
import math
import numpy as np
import common
import multiprocessing as mp
import random

RNG = np.random.default_rng()

# TQ the Tabu Queue
class TQ(object):
    def __init__(self, size):
        self.size = size
        self.queue = []

def Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, iterSolID, N, items, nodeTorque, k, cfg, lock):

    for i, _ in enumerate(iterPallets):

        iterScore -= iterPallets[i].PCS

        arr0 = [j for j, a in enumerate(iterItemsDict["mpItems"])                if a == 0]
        arr1 = [j for j, a in enumerate(iterSolDict["solMatrix"][N*i:N*i+(N-1)]) if a == 1]

        if len(arr0) > 0 and len(arr1) > 0:
            id0 = random.choice(arr0) # not in solution
            id1 = random.choice(arr1) # in this pallet

            iterPallets[i].popItem(items[id1], nodeTorque, iterSolDict, N, iterItemsDict)
            
            if iterPallets[i].isFeasible(items[id0], 1.0, k, nodeTorque, cfg, iterItemsDict, lock):
                iterPallets[i].putItem(items[id0], nodeTorque, iterSolDict, N, iterItemsDict, lock)
            else:
                iterPallets[i].putItem(items[id1], nodeTorque, iterSolDict, N, iterItemsDict, lock)


        iterScore += iterPallets[i].PCS

        iterSolID += int(iterPallets[i].PCS + iterPallets[i].PCW + iterPallets[i].PCV)


def Solve(pallets, items, cfg, k, secBreak, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items)
    M = len(pallets)

    # Tabu Queue size: 2% the size of the problem
    len_tq = math.ceil( float(N * M) / 50 )
    tq = TQ(len_tq)

    # the bigger the problem less iterations (244 - 27 iterations)
    numIter = math.ceil( 600_000.0 / float(len_tq) )

    print(f"\nTabu Search for ACLP+RPDP\n")        

    lock  = mp.Lock() # for use in parallel mode

    pallets.sort(key=lambda x: abs(x.D)) # sort pallets ascendent by CG distance

    print(f"{len(items)} items  {len(pallets)} pallets")

    initScore = 0.0 # G

    solID = 0

    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, nodeTorque, solDict, cfg, 0.9, itemsDict, lock)
        initScore += pallets[i].PCS

        solID += int(pallets[i].PCS + pallets[i].PCW + pallets[i].PCV)

    print(f"Greedy initial score {initScore}")

    bestScore = initScore # G*

    # TS phase -------------------------------------------------------------------

    stagnant = 0
    while stagnant <= 3 and (time.perf_counter() - startTime) < secBreak:

        # re-start with the greedy solution
        localScore     = initScore
        localSolID     = solID
        localSolDict   = common.copySolDict(solDict)
        localItemsDict = common.copyItemsDict(itemsDict)
        localPallets   = common.copyPallets(pallets)

        iterScore      = initScore
        iterSolID      = solID
        iterSolDict    = common.copySolDict(solDict)
        iterItemsDict  = common.copyItemsDict(itemsDict)
        iterPallets    = common.copyPallets(pallets)        

        for _ in range(numIter): # local solution iterations

            if ((time.perf_counter() - startTime) > secBreak):
                break            

            # Transform makes a random elementary transformation in each pallet
            Transform(iterPallets, iterItemsDict, iterSolDict, iterScore, iterSolID, N, items, nodeTorque, k, cfg, lock)

            if iterScore > localScore and iterSolID not in tq.queue:

                localScore     = iterScore
                localSolDict   = common.copySolDict(iterSolDict)
                localItemsDict = common.copyItemsDict(iterItemsDict)
                localPallets   = common.copyPallets(iterPallets)
                localSolID     = iterSolID            

        if localSolID not in tq.queue:
            tq.queue.append(localSolID)
        
        if len(tq.queue) > len_tq:
            tq.queue.pop(0)   

        # try to improve here
        # for i, _ in enumerate(pallets):
            # common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock)


        if localScore > bestScore:
            
            bestScore = localScore
            solDict   = common.copySolDict(localSolDict)
            itemsDict = common.copyItemsDict(localItemsDict)
            pallets   = common.copyPallets(localPallets)

            stagnant = 0
        else:
            stagnant += 1

 
if __name__ == "__main__":

    print("----- Please execute the main py file -------------")