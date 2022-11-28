import methods as mno
import multiprocessing as mp
import shims

def shimsEnqueue( outQueue,          p, notInSol, sol, limit, numItems, maxTorque, k):
    outQueue.put( shims.getBestShims(p, notInSol, sol, limit, numItems, maxTorque, k) )

# parallel Shims based on different limits
def Solve(pallets, items, cfg, k, limit): # items include kept on board

    print(f"\nParallel Shims for ACLP+RPDP")

    numItems   = len(items)
    numPallets = len(pallets)

    edges = mno.mountEdges(pallets, items, cfg)

    sol = mno.Solution(edges, pallets, items, limit, cfg, k)

    notInSol = [ [] for _ in range(len(pallets)) ]

    procs = [None for _ in pallets]
    outQueue = mp.Queue()

    for i, p in enumerate(pallets):

        notInSol[i] = [e for e in sol.Edges if e.Pallet.ID == p.ID and not e.InSol  ]

        procs[i] = mp.Process( target=shimsEnqueue,args=( outQueue, p, notInSol[i], sol, limit, numItems, cfg.maxTorque, k  ) )
        
    for p in procs:
        p.start()
    
    for _ in procs:
        BestShims = outQueue.get()
        for e in BestShims:
            sol.putInSol(e)

    # local search
    # counter = 0
    # for e in sols[bestID].Edges:
    #     if not e.InSol and sol.isFeasible(e, 1.03, cfg, k):
    #         sol.putInSol(e)
    #         counter += 1
    # print(f"{counter} extra edges put in the best solution")

    return mno.getSolMatrix(sol.Edges, numPallets, numItems)
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
