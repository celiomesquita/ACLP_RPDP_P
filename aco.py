
import time
import numpy as np
import methods
import copy
import aco_common as ac

def Build( sol, edges, cfg, k ):

    # neighborhood edges are a heritage from previous ants
    nbhood = methods.edges_copy(edges)
    # prepare for the random proportional selection in selectInNbhood method
    attracts = np.full(len(edges), 0.0)

    attractSum = 0.0
    numNb = 0
    for ce in nbhood:
        if ce.Item.P == -1 and not ce.InSol: # items only
            attracts[ce.ID] = ce.Attract
            attractSum += ce.Attract
            numNb += 1
        else:
            nbhood[ce.ID].Attract = 0   

    startTime = time.perf_counter()

    # builds or completes a solution by exploring the entire items neighborhood
    while numNb and (time.perf_counter() - startTime) < methods.SEC_BREAK:

        numNb -= 1              

        # select a candidate edge from the neighborhood by a random proportional selection
        # the last argument 0.0 means no greediness
        ce, attractSum = ac.selectInNbhood(nbhood, attracts, attractSum, 0.0)

        if sol.isFeasible(ce, 1.00, cfg, k):
            sol.includeEdge(ce)
                    

def Solve( pallets, items, startTime, cfg, k):  # items include kept on board

    print("\nSerial Ant Colony Optimization for ACLP+RPDP")

    edges = methods.mountEdges(pallets, items, cfg, k)

    # initialize the best solution                  1.0 totally greedy
    Gbest = methods.Solution(edges, pallets, items, 1.0, cfg, k)
 
    max = 3
    stagnant = 0
    numAnts = 0
    # while stagnant <= max and (time.perf_counter()- startTime) < methods.SEC_BREAK:
    for _ in range(ac.NANTS):

        Glocal = copy.copy(Gbest)
   
        for _ in range(ac.NANTS): # sequential ants

            numAnts += 1

            Gant = methods.Solution(edges, pallets, items, 0.5, cfg, k)

            Build( Gant, edges, cfg, k )
                    
            if Gant.S > Glocal.S:
                Glocal = copy.copy(Gant)

        if Glocal.S > Gbest.S:
            Gbest = copy.copy(Glocal)
            stagnant = 0
            ac.updatePheroAttract(Gant.S, Gbest.S, edges, numAnts)
        else:
            stagnant += 1

    print(f"Number of ants used: {numAnts}")

    # decision matrix for which items will be put in which pallet
    X = np.zeros((len(pallets),len(items))) # needs 2 parenthesis
    for e in Gbest.Edges:
        if e.InSol == 1:
            X[e.Pallet.ID][e.Item.ID] = 1

    return X

if __name__ == "__main__":

    print("----- Please execute the main py file -------------")