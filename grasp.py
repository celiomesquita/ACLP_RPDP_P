
import common
import time
import math
import numpy as np
import multiprocessing as mp
import common

RNG = np.random.default_rng()

def pickRandomEdge(edges):
    n = len(edges)
    i = math.floor(RNG.random()*n)
    e = edges[i]
    edges.pop(i)
    return e

# RCL the Restricted Candidates List
class RCL(object):
    def __init__(self, size, edges): # edges is received sorted by eta
        self.size = size
        self.list = []
        if len(edges) >= size:
            for _ in range(size):
                # greedily assemble the RCL.
                e = edges[0]
                edges.pop(0)
                self.list.append(e)

def Solve(pallets, items, cfg, pi, k, secBreak, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items)
    M = len(pallets)

    print(f"\nGRASP for ACLP+RPDP ({pi}-{k})")        

    lock  = mp.Lock() # for use in parallel mode

    print(f"{len(items)} items  {len(pallets)} pallets")

    initScore = 0.0 # G

    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, nodeTorque, solDict, cfg, 0.95, itemsDict, lock)
        initScore += pallets[i].PCS

    bestScore = initScore # G*

    # initialize the edges between pallets and items
    Ek = [None] * N * M
    id = 0
    for p in pallets:
        for it in items:
            Ek[id] = common.Edge(id, p, it)
            id += 1    

    # the most attractive edges come first
    Ek.sort(key=lambda  x: x.Attract, reverse=True)

    # RCL size: 2% the size of the problem
    L_rcl = math.ceil( float(N * M) / 50 )

    Etemp  = []
    Eprime = [] 

    bestScore = initScore

    stagnant = 0
    while stagnant <= 3 and (time.perf_counter() - startTime) < secBreak:

        localScore     = initScore
        localSolDict   = dict(solDict)
        localItemsDict = dict(itemsDict)

        iterScore      = initScore
        iterSolDict    = dict(solDict)
        iterItemsDict  = dict(itemsDict)

        Eprime = Ek
        rcl = RCL(L_rcl, Eprime) # edges are drawed from Eprime

        while 1:

            if ((time.perf_counter() - startTime) > secBreak):
                break 

            while 1:

                if ((time.perf_counter() - startTime) > secBreak):
                    break 

                if len(rcl.list) < L_rcl:
                    break

                ce = pickRandomEdge(rcl.list)

                for e in rcl.list:
                    Etemp.append(e)

                # try to insert this candidate edge

                if ce.Pallet.isFeasible(ce.Item, 1.0, k, nodeTorque, cfg, iterItemsDict, lock):

                    ce.Pallet.putItem(ce.Item, nodeTorque, iterSolDict, N, iterItemsDict, lock)

                    iterScore += ce.Pallet.PCS

                if iterScore > localScore:

                    localScore     = iterScore
                    localSolDict   = dict(iterSolDict)
                    localItemsDict = dict(iterItemsDict)

                rcl = RCL(L_rcl, Eprime) 

            if len(Etemp) < L_rcl:
                break 
            
            Eprime = Etemp
            rcl = RCL(L_rcl, Eprime)

        if localScore > bestScore:

            bestScore = localScore

            solDict   = dict(localSolDict)
            itemsDict = dict(localItemsDict)

            stagnant = 0
        else:
            stagnant += 1


    for i, _ in enumerate(pallets):               
        common.fillPallet(pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock)

if __name__ == "__main__":

    print("----- Please execute the main py file -------------")