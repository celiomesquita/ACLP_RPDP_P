
import common
from time import time
import math
import numpy as np
from multiprocessing import Queue, Process
import common

RNG = np.random.default_rng()

def pickFirstEdge(vector):
    e = vector[0]
    vector.pop(0)
    return e

def pickRandomEdge(vector):
    n = len(vector)
    i = math.floor(RNG.random()*n)
    e = vector[i]
    vector.pop(i)
    return e

# RCL the Restricted Candidates List
class RCL(object):
    def __init__(self, size, nbhood): # nbhood is received sorted by eta
        self.size = size
        self.list = []
        if len(nbhood) >= size:
            for _ in range(size):
                # greedilly assemble the RCL
                e = pickFirstEdge(nbhood)
                self.list.append(e)


def Solve( pallets, items, startTime, cfg, k):  # items include kept on board

    print("\nGRASP for ACLP+RPDP")

    Gbest = common.Solution(pallets, items, 1.0, cfg, k, False) # False: greedy solution

    # initialize the edges between pallets and items
    Ek = [None] * len(pallets) * len(items)
    id = 0
    for p in pallets:
        for it in items:
            Ek[id] = common.Edge(id, p, it, cfg)
            id += 1    

    Ek.sort(key=lambda  x: x.Heuristic, reverse=True)

    L_rcl = math.floor( len(Ek) / (len(Gbest.Edges)) )

    Etemp  = []
    Eprime = [] 

    stagnant = 0
    while stagnant < 3 and (time() - startTime) < common.SEC_BREAK:

        Glocal = Gbest

        Eprime = Ek
        rcl = RCL(L_rcl, Eprime) # edges are drawed from Eprime

        while 1:

            if ((time() - startTime) > common.SEC_BREAK):
                break

            while 1:

                if ((time() - startTime) > common.SEC_BREAK):
                    break

                if len(rcl.list) < L_rcl:
                    break

                ce = pickRandomEdge(rcl.list)

                for e in rcl.list:
                    Etemp.append(e)

                if Glocal.isFeasible(ce, 1.0, cfg, k): # Check all constraints
                    Glocal.AppendEdge(ce)
                    if ce in Glocal.Nbhood:
                        Glocal.Nbhood.remove(ce) 

                rcl = RCL(L_rcl, Eprime) 

            if len(Etemp) < L_rcl:
                break 
            
            Eprime = Etemp
            rcl = RCL(L_rcl, Eprime)

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
        else:
            stagnant += 1

    # decision matrix for which items will be put in which pallet
    X = [[0 for _ in np.arange(len(items))] for _ in np.arange(len(pallets))]
    for e in Gbest.Edges:
        X[e.Pallet.ID][e.Item.ID] = 1

    return X


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")