import methods as mno
import numpy as np
import multiprocessing as mp
import shims

def shimsQueue(           edges, pallets, items, limit, cfg, k, outQueue ):
    outQueue.put( Compute(edges, pallets, items, limit, cfg, k) )

def Compute(edges, pallets, items, limit, cfg, k) :

    sol = mno.Solution(edges, pallets, items, limit, cfg, k)

    #edges not in sol groupped by pallets
    notInSol = [ [] for _ in range(len(pallets)) ]

	# get the best shim that fills the slack
    for p in (pallets):

        notInSol[p.ID] = [e for e in sol.Edges if e.Pallet.ID == p.ID and not e.InSol  ]

		# get the best shim of edges             greedy limit
        BestShims = shims.getBestShims(p, notInSol[p.ID], sol, limit, len(items), cfg.maxTorque, k)

        # move the best shim of edges to solution
        for e in BestShims:
            sol.putInSol(e)
            notInSol[e.Pallet.ID].remove(e)

    # local search: Maybe still exists any edge that fit in solution
    for nis in notInSol: # for each row
        for ce in nis:
            if sol.isFeasible(ce, 1.0, cfg, k):
                sol.putInSol(ce)

    return sol


def Solve(pallets, items, cfg, k, minLim, numCPU): # items include kept on board

    print(f"\nParallel Shims for ACLP+RPDP")

    numItems   = len(items)
    numPallets = len(pallets)

    edges = mno.mountEdges(pallets, items, cfg)

	# pallets closer to the CG are completed first
    pallets.sort(key=lambda x: abs(x.D), reverse=False)      

    procs = [None for _ in range(numCPU)]
    outQueue = mp.Queue()

    for i, p in enumerate(procs):

        limit = minLim + (1.0 - minLim)*(i+1)/numCPU

        # create a child process for each limit
        procs[i] = mp.Process( target=shimsQueue,args=( edges, pallets, items, limit, cfg, k, outQueue  ) )
    for p in procs:
        p.start()
        
    sols = [outQueue.get() for _ in procs]

    bestScore = 0
    bestID    = 0
    for i, sol in enumerate(sols):
        if sol.S > bestScore:
            bestScore = sol.S
            bestID = i

    return mno.getSolMatrix(sols[bestID].Edges, numPallets, numItems)
        
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
