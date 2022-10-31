import methods as mno
import numpy as np

def Solve(pallets, items, cfg, k): # items include kept on board

    print("\nGreedy Heuristic for ACLP+RPDP")

    edges = mno.mountEdges(pallets, items, cfg, k)

    numItems   = len(items)
    numPallets = len(pallets)

    sol = mno.Solution(edges, pallets, items, 1.0, cfg, k, True) # True: with torque constraints

    return mno.getSolMatrix(sol.Edges, numPallets, numItems)
        
if __name__ == "__main__":

    print("----- Please execute module main_test -----")
