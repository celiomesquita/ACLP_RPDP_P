import methods
import numpy as np

def Solve(pallets, items, cfg, k): # items include kept on board

    print("\nGreedy Heuristic for ACLP+RPDP")

    edges = methods.mountEdges(pallets, items, cfg, k)

    numItems   = len(items)
    numPallets = len(pallets)

    sol = methods.Solution(edges, pallets, items, 1.0, cfg, k)

    X = np.zeros((numPallets,numItems)) # needs 2 parenthesis

    for e in sol.Edges:
        if e.InSol == 1:
            X[e.Pallet.ID][e.Item.ID] = 1

    return X
        
if __name__ == "__main__":

    print("----- Please execute module main_test -----")
