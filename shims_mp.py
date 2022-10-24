import methods
import numpy as np
import math
import shims
import multiprocessing as mp
import time
import copy

def comp_queue(         edges, pallets, items, limit, cfg, k, q):
    q.put(shims.Compute(edges, pallets, items, limit, cfg, k))

def Solve(pallets, items, cfg, k): # items include kept on board

    # t0 = time.perf_counter()

    print(f"\nShims_mp, a new multiprocessing Heuristic for ACLP+RPDP")

    edges = methods.mountEdges(pallets, items, cfg, k)

    numItems   = len(items)
    numPallets = len(pallets)

    # as greater is the number of edges, closest to 1 is the limit

    # NUM = 35
    NUM = 2200

    if methods.DATA == "data50":
        NUM *= 2
    if methods.DATA == "data100":
        NUM *= 3

    step = math.floor(NUM/methods.NCPU)

    num = []

    last1 = NUM
    last2 = NUM

    for i in range(methods.NCPU):
        if i % 2 == 0:
            # print(f"{last1}\t", end="")
            num.append(last1)
            last1 += step
        else:
            last2 -= step
            # print(f"{last2}\t", end="")
            num.append(last2)

    print()
    
    lims = []
    for i in range(methods.NCPU):
        limit = 1.0 - num[i]/float(numItems*numPallets)
        lims.append(limit)
        # print(f"{limit:.2f}\t", end="")
    # print()

    procs = [None for _ in range(methods.NCPU)]
    out_queue = mp.Queue()

    for i in range(methods.NCPU):
        procs[i] = mp.Process(target=comp_queue,args=(edges, pallets, items, lims[i], cfg, k, out_queue))
    for p in procs:
        p.start()

    # t1 = time.perf_counter()

    # wait for all computations to finish
    sols = [out_queue.get() for _ in procs]

    # t2 = time.perf_counter()

    best = 0
    edges = []
    for sol in sols:
        if sol.Heuristic > best:
        # if sol.S > best:
            best = sol.Heuristic
            # best = sol.S
            print(sol.S, sol.Limit)
            edges = methods.edges_copy(sol.Edges)

    # decision matrix for which items will be put in which pallet
    X = np.zeros((numPallets,numItems)) # needs 2 parenthesis
    for e in edges:
        X[e.Pallet.ID][e.Item.ID] = 1

    # print(f"durations\t{t1-t0:.3f}\t{t2-t1:.3f}\tlim: {lims[0]:.2f}\n")

    return X
        
if __name__ == "__main__":

    print("----- Please execute module main_test -----")