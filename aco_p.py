import time
import math
import numpy as np
import methods as mno
import multiprocessing as mp
import aco

# Process-based parallelism

def antSolve(Gant, cfg, k):
    Nbhood   = [ce for ce in Gant.Edges if not ce.InSol]
    attracts = [ce.Attract for ce in Nbhood]
    while Nbhood:
        ce = aco.pickFromNbhood(Nbhood, attracts)
        if Gant.isFeasible(ce, 1.0, cfg, k):
            Gant.putInSol(ce)            
    return Gant

def antQueue( Gant, cfg, k, outQueue ):
    outQueue.put( antSolve(Gant, cfg, k) )

def Solve( pallets, items, startTime, cfg, k, limit):  # items include kept on board

    print("\nMultiprocess Ant Colony Optimization for ACLP+RPDP")

    antsField = mno.mountEdges(pallets, items, cfg)

    # initialize the best solution so far           1.0 totally greedy
    Gbest = mno.Solution(antsField, pallets, items, 1.00, cfg, k)

    initialS = Gbest.S

    numPallets = len(pallets)
    numItems   = len(items)
    numAnts = 0
    stagnant = 0
    improvements = 0
   
    while stagnant < aco.NANTS and (time.perf_counter() - startTime) < mno.SEC_BREAK:    

        Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

        procs = [None for _ in range(aco.NANTS)]
        outQueue = mp.Queue()
        for i, p in enumerate(procs):
            procs[i] = mp.Process( target=antQueue,args=( Glocal, cfg, k, outQueue  ) )
        for p in procs:
            numAnts+=1
            p.start()
        sols = [outQueue.get() for _ in procs]

        for Gant in sols:
            if Gant.S > Glocal.S:
                Glocal = Gant

        if Glocal.S > Gbest.S:
            Gbest = Glocal
            stagnant = 0
            improvements += 1
        else:
            stagnant += 1

        aco.updatePheroAttract(Glocal.S, Gbest.S, antsField, aco.NANTS)

    if initialS > 0:
        print(f"Used {numAnts} ants | ratio {Glocal.S/initialS:.3f} | {improvements} improvements")

    return mno.getSolMatrix(Gbest.Edges, numPallets, numItems)

        
if __name__ == "__main__":

    print("----- Please execute module main -----")
