import methods as mcuda
import numpy as np

def Solve(pallets, items, cfg, k): # items include kept on board

    print("\nGreedy Heuristic for ACLP+RPDP")

    Nbhood = mcuda.mountEdges(pallets, items, cfg, k)

    numItems   = len(items)
    numPallets = len(pallets)

    sol = mcuda.Solution(Nbhood, pallets, items, 1.0, cfg, k)

    return mcuda.getSolMatrix(sol.Edges, numPallets, numItems)
        
if __name__ == "__main__":

    print("----- Please execute module main_test -----")
