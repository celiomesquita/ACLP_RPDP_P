import time
import methods as mno
import multiprocessing as mp
# import threading as thrd

import aco

# Process-based parallelism

def antSolve(Gant, cfg, k, antsField, bestScore, mpAttract):
    
    Nbhood   = [ce for ce in Gant.Edges if not ce.InSol]
    attracts = [ce.Attract for ce in Nbhood]

    # for i, e in enumerate(Nbhood): # https://superfastpython.com/multiprocessing-shared-ctypes-in-python/
    #     attracts[i] = mpAttract[e.ID]

    while Nbhood:

        ce = aco.pickFromNbhood(Nbhood, attracts)

        if Gant.isFeasible(ce, 1.0, cfg, k):
            Gant.putInSol(ce)
    
    aco.updatePheroAttract(Gant.S, bestScore, antsField, mpAttract)

    return Gant

def enqueue( antsQueue,     Gant, cfg, k, antsField, bestScore, mpAttract ):
    antsQueue.put( antSolve(Gant, cfg, k, antsField, bestScore, mpAttract) )

def publish(antsQueue, procs, limits, antsField, pallets, items, cfg, k, bestScore, mpAttract ):

    for i, _ in enumerate(procs):

        limit = limits[i] # greedy limit

        Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

        # create a child process for each ant
        procs[i] = mp.Process( target=enqueue, args=( antsQueue, Glocal, cfg, k, antsField, bestScore, mpAttract ) ) # faster than thread
        # procs[i] = thrd.Thread( target=enqueue, args=( antsQueue, Glocal, cfg, k, antsField, bestScore ) )
    
        procs[i].start()
         

def subscribe(antsQueue, procs, startTime, secBreak):

    sols = [antsQueue.get() for _ in procs if time.perf_counter() - startTime < secBreak ]
    for p in procs:
        p.terminate()  
    antsQueue.close()
    return sols  

def Solve( pallets, items, startTime, cfg, k, minLim, numProcs, secBreak):  # items include kept on board

    print("\nParallel Ant Colony Optimization for ACLP+RPDP")

    antsField, mpAttract = mno.mountEdges(pallets, items, cfg)

    # set of unique greedy limit values
    limits = mno.getLimits(minLim, numProcs) 

    # initialize the best solution so far           1.0 totally greedy
    Gbest = mno.Solution(antsField, pallets, items, 0.95, cfg, k)

    initialS  = Gbest.S

    numPallets = len(pallets)
    numItems   = len(items)
    numAnts = 0
    stagnant = 0
    improvements = 0
   
    while stagnant <= 3 and (time.perf_counter() - startTime) < secBreak:    

        procs = [None for _ in range(numProcs)]

        numAnts += numProcs

        antsQueue = mp.Queue()

        publish(antsQueue, procs, limits, antsField, pallets, items, cfg, k, Gbest.S, mpAttract)

        sols = subscribe(antsQueue, procs, startTime, secBreak)

        for Gant in sols:

            if Gant.S > Gbest.S:
                Gbest = Gant
                stagnant = 0
                improvements += 1
            else:
                stagnant += 1

            aco.updatePheroAttract(Gant.S, Gbest.S, antsField, mpAttract)

    if initialS > 0:
        print(f"\nUsed {numAnts} ants | ratio {Gbest.S/initialS:.3f} | {improvements} improvements\n")

    return mno.getSolMatrix(Gbest.Edges, numPallets, numItems)

        
if __name__ == "__main__":

    print("----- Please execute module main -----")
