import time
import methods as mno
import multiprocessing as mp
import aco

# Process-based parallelism

def getLimits(minLim, numProcs): # pid: 0 - numProcs-1

    limitSet = set()

    maxLim = 0.98

    delta = (maxLim - minLim)/(numProcs+1)

    # set of unique limit values
    for i in range(numProcs):
        limitSet.add(minLim + (i+1)*delta)

    # return list(limitSet)
    return [*limitSet, ] # a bit faster

def antSolve(Gant, cfg, k, antsField, bestScore):
    
    Nbhood   = [ce for ce in Gant.Edges if not ce.InSol]
    attracts = [ce.Attract for ce in Nbhood]

    while Nbhood:

        ce = aco.pickFromNbhood(Nbhood, attracts)

        if Gant.isFeasible(ce, 1.0, cfg, k):
            Gant.putInSol(ce)
    
    aco.updatePheroAttract(Gant.S, bestScore, antsField)

    return Gant

def enqueue( antsQueue,     Gant, cfg, k, antsField, bestScore ):
    antsQueue.put( antSolve(Gant, cfg, k, antsField, bestScore) )

def publish(antsQueue, procs, limits, antsField, pallets, items, cfg, k, bestScore ):

    for i, p in enumerate(procs):

        limit = limits[i] # greedy limit

        Glocal = mno.Solution(antsField, pallets, items, limit, cfg, k)

        # create a child process for each ant
        procs[i] = mp.Process( target=enqueue, args=( antsQueue, Glocal, cfg, k, antsField, bestScore ) )
    
    numAnts = 0    
    for p in procs:
        numAnts+=1
        p.start()

    return numAnts


def subscribe(antsQueue, procs, startTime, secBreak):

    sols = [antsQueue.get() for _ in procs if time.perf_counter() - startTime < secBreak ]
    
    for p in procs:
        p.terminate()  

    antsQueue.close()

    return sols  

def Solve( pallets, items, startTime, cfg, k, minLim, numProcs, secBreak):  # items include kept on board

    print("\nMultiprocess Ant Colony Optimization for ACLP+RPDP")

    antsField = mno.mountEdges(pallets, items, cfg)

    # set of unique greedy limit values
    limits = getLimits(minLim, numProcs) 

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

        antsQueue = mp.Queue()

        numAnts += publish(antsQueue, procs, limits, antsField, pallets, items, cfg, k, Gbest.S)

        sols = subscribe(antsQueue, procs, startTime, secBreak)

        for Gant in sols:

            if Gant.S > Gbest.S:
                Gbest = Gant
                stagnant = 0
                improvements += 1
            else:
                stagnant += 1

            aco.updatePheroAttract(Gant.S, Gbest.S, antsField)

    if initialS > 0:
        print(f"\nUsed {numAnts} ants | ratio {Gbest.S/initialS:.3f} | {improvements} improvements\n")

    return mno.getSolMatrix(Gbest.Edges, numPallets, numItems)

        
if __name__ == "__main__":

    print("----- Please execute module main -----")
