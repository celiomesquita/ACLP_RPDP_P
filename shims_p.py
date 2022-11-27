import methods as mno
import multiprocessing as mp
import shims

def shimsEnqueue(outQueue, edges, pallets, items, limit, cfg, k ):
    outQueue.put( Compute( edges, pallets, items, limit, cfg, k) )

def Compute(edges, pallets, items, limit, cfg, k) :

    sol = mno.Solution(edges, pallets, items, limit, cfg, k)

    #edges not in sol groupped by pallets
    notInSol = [ [] for _ in range(len(pallets)) ]

	# get the best shim that fills the slack
    for p in (pallets):

        notInSol[p.ID] = [e for e in sol.Edges if e.Pallet.ID == p.ID and not e.InSol  ]

		# get the best shim of edges             greedy limit
        BestShims = shims.getBestShims(p, notInSol[p.ID], sol, limit, len(items), cfg.maxTorque, k)
 
        for e in BestShims:
            sol.putInSol(e)

    return sol

# parallel Shims based on different limits
def Solve(pallets, items, cfg, k, limit, numProcs): # items include kept on board

    print(f"\nParallel Shims for ACLP+RPDP")

    numItems   = len(items)
    numPallets = len(pallets)

    edges = mno.mountEdges(pallets, items, cfg)

	# pallets closer to the CG are completed first
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

    procs = [None for _ in range(numProcs)]
    outQueue = mp.Queue()

    for i, p in enumerate(procs):

        # create a child process for each limit
        procs[i] = mp.Process( target=shimsEnqueue,args=( outQueue, edges, pallets, items, limit, cfg, k  ) )
        
    for p in procs:
        p.start()
        
    sols = [outQueue.get() for _ in procs]

    bestScore = 0
    bestID    = 0
    for i, sol in enumerate(sols):
        if sol.S > bestScore:
            bestScore = sol.S
            bestID = i

    # local search
    counter = 0
    for e in sols[bestID].Edges:
        if not e.InSol and sol.isFeasible(e, 1.03, cfg, k):
            sol.putInSol(e)
            counter += 1

    print(f"{counter} extra edges put in the best solution")

    return mno.getSolMatrix(sols[bestID].Edges, numPallets, numItems)
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
