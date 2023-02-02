import common
import numpy as np
import time
import multiprocessing as mp
import os
import math

import mpShims
import mpACO
import optcgcons
import mipGRB

shimsThreshold = 0.25 # best solume threshold
acoThreshold   = 0.25

def solveTour(scenario, inst, pi, tour, method, pallets, cfg, secBreak, surplus):
    """
    Solves one tour
    """
    print(f"----- Tour {pi},", end='')

    for node in tour.nodes:
        print(f" {node.ICAO}", end='')
    print()

    # a matrix for all consolidated in the tour
    consol = [
                [ common.Item(-1, -2, 0, 0, 0., -1, -1) # an empty consolidated
                for _ in tour.nodes ]
                for _ in pallets # tour consolidated for each pallet
            ]

    base = len(tour.nodes)-1

    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

        for i, p in enumerate(pallets):
            pallets[i].reset(cfg.numNodes)

        # solution global torque to be shared and changed by all pallets concurrently
        solTorque = mp.Value('d', 0.0) # a multiprocessing double type variable
        
        # initialize- the accumulated values
        sNodeAccum = 0.
        wNodeAccum = 0.
        vNodeAccum = 0.

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # load items parameters from this node and problem instance, that go to unnatended
        items = common.loadNodeItems(scenario, inst, node, unattended, surplus)

        if k > 0 and k < base: # not in the base

            # load consolidated generated in the previous node
            # prevNode = tour.nodes[k-1]
            # cons = common.loadNodeCons(surplus, scenario, inst, pi, prevNode, numItems )

            cons = []
            for i, _ in enumerate(pallets): # previous node consolidated
                    cons.append( consol[i][k-1] )

            # consolidated contents not destined to this point are kept on board ...
            kept = []
            cid = N
            for c in cons:
                if c.To in unattended:
                    c.ID = cid
                    kept.append(c) #... and included in the items set
                    cid += 1

            # Optimize consolidated positions to minimize CG deviation.
            # Pallets destinations are also set, according to kept on board in new positions
            # Kept P is not -2 anymore, but the pallet ID.
            if len(kept) > 0:
                # optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)

                # N: number of items to embark
                # put the consolidated on their assgined pallets (OptCGCons)
                for c in kept:
                    for i, p in enumerate(pallets):
                        if c.P == p.ID:
                            pallets[i].putConsol( c, solTorque)

                            # update the consolidated of the current node "k"
                            consol[i][k].ID  = j+N
                            consol[i][k].Frm = node.ID
                            consol[i][k].To  = pallets[i].Dest[k]
                            consol[i][k].W  += c.W
                            consol[i][k].V  += c.V
                            consol[i][k].S  += c.S

                            # update the accumulated values
                            sNodeAccum += c.S
                            wNodeAccum += c.W
                            vNodeAccum += c.V
                            tour.score += c.S

        # set pallets destinations with items and consolidated to be delivered
        if k < base: # except when the current node is the base on returning
            common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)
        else:
            return # skip solving because it's the base on returning

        startNodeTime = time.perf_counter()

        # to control solution items
        M = len(pallets)
        N = len(items)
        solMatrix = mp.Array('i', [0 for _ in np.arange(N*M)] )
        mpItems   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

        solDict   = dict(solMatrix=solMatrix)
        itemsDict = dict(mpItems=mpItems)



        if method == "mpShims":
            mpShims.Solve(pallets, items, cfg, k, shimsThreshold, secBreak, "p", solTorque, solDict, itemsDict)

        if method == "Shims":            
            mpShims.Solve(pallets, items, cfg, k, shimsThreshold, secBreak, "s", solTorque, solDict, itemsDict)         

        if method == "mpACO":       
            mpACO.Solve(pallets,   items, cfg, k, acoThreshold,   secBreak, "p", solTorque, solDict, itemsDict) 

        if method == "ACO":       
            mpACO.Solve(pallets,   items, cfg, k, acoThreshold,   secBreak, "s", solTorque, solDict, itemsDict) 

        if method == "GRB":       
            mipGRB.Solve(pallets,  items, cfg, k,            secBreak,      solTorque, solDict, itemsDict) 

        nodeElapsed = time.perf_counter() - startNodeTime

        tour.elapsed += nodeElapsed

        Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)

        for i, row in enumerate(Y):
            for j, X_ij in enumerate(row):
                if X_ij:
                    # mount this node "k" consolidated
                    consol[i][k].ID  = j+N
                    consol[i][k].Frm = node.ID
                    consol[i][k].To  = pallets[i].Dest[k]                    
                    consol[i][k].W += items[j].W
                    consol[i][k].V += items[j].V
                    consol[i][k].S += items[j].S
                    # totalize parameters of this solution
                    sNodeAccum += float(items[j].S)
                    wNodeAccum += float(items[j].W)
                    vNodeAccum += float(items[j].V)

                    tour.score += items[j].S  

        epsilom = solTorque.value/cfg.maxTorque
        tour.cost *= ( 1.0 + abs(epsilom)/20.0 )

        print(f"----- node {node.ICAO},", end='')
        print(f" score {tour.score:.0f}, cost {tour.cost:.0f} -----\n")

        consNodeT = [None for _ in pallets]        
        for i, p in enumerate(pallets):
            consNodeT[i] = consol[i][k]

        vol = vNodeAccum/cfg.volCap
        wei = wNodeAccum/cfg.weiCap

        # write consolidated contents from this node in file
        common.writeNodeCons(scenario, instance, consNodeT, pi, node, surplus, epsilom, wei, vol)
            
# end of solveTour 

def writeAvgResults(method, scenario, line, surplus):

    dirname = f"./results/{surplus}/{method}_{scenario}"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass  

    fname = f"{dirname}/{method}_{scenario}_avg.txt"

    writer = open(fname,'w+') # + creates the file, if not exists

    try:
        writer.write(line)
    finally:
        writer.close()  

if __name__ == "__main__":

# --- Home ---
#   Shims_6, 16.01, 439, 123 tours, data50, Worst tour time: 4.58
# 
#     ACO_6, 5.72, 4819, 123 tours, data50, Worst tour time: 42.37
#   mpACO_6, 5.74, 3734, 123 tours, data50, Worst tour time: 31.86
#     GRB_6 16.01, 2529, 123 tours, data50, Worst tour time: 23.61

#   Shims_2, 16.00,  3, 2 tours, data50, Worst tour time:  2.16
# 
#     ACO_2,  7.02, 38, 2 tours, data50, Worst tour time: 19.99
#   mpACO_2,  7.00, 29, 2 tours, data50, Worst tour time: 15.23
#     GRB_2, 10.62, 25, 2 tours, data50, Worst tour time: 12.75
 
# --- work ---
#    Shims_2, 16.00,  2, 2 tours, data50, Worst tour time:  0.97
#  mpShims_2, 21.50,  1, 2 tours, data50, Worst tour time:  0.50
#      ACO_2,  7.02, 22, 2 tours, data50, Worst tour time: 11.09
#    mpACO_2,  6.99, 18, 2 tours, data50, Worst tour time:  8.92
#      GRB_2, 10.84, 16, 2 tours, data50, Worst tour time:  8.06

#   Shims_1, 12.17,  1, 2 tours, data50, Worst tour time: 0.19
# mpShims_1, 13.88,  1, 2 tours, data50, Worst tour time: 0.23
#     ACO_1,  8.54, 12, 2 tours, data50, Worst tour time: 5.73
#   mpACO_1,  8.57, 11, 2 tours, data50, Worst tour time: 5.51
#     GRB_1, 15.86, 10, 2 tours, data50, Worst tour time: 5.28

# ----- Work without optCGcons -----
#   Shims_1, 14.89,  1, 2 tours, data50, Worst tour time: 0.12
# mpShims_1, 17.77,  1, 2 tours, data50, Worst tour time: 0.16
#     GRB_1, 12.11, 10, 2 tours, data50, Worst tour time: 5.35

#   Shims_2, 13.12, 2, 2 tours, data50, Worst tour time: 0.97
# mpShims_2, 18.61, 1, 2 tours, data50, Worst tour time: 0.48
#     GRB_2, 8.47, 16, 2 tours, data50, Worst tour time: 8.03

#   Shims_3, 15.20,  4, 3 tours, data50, Worst tour time:  1.17
# mpShims_3, 19.72,  2, 3 tours, data50, Worst tour time:  0.64
#     GRB_3, 10.43, 31, 3 tours, data50, Worst tour time: 10.96

#   Shims_4, 14.18, 13, 10 tours, data50, Worst tour time:  1.38
# mpShims_4, 17.96,  8, 10 tours, data50, Worst tour time:  0.83
#     GRB_4, 9.85, 109, 10 tours, data50, Worst tour time: 11.08

#   Shims_5, 12.93, 38, 24 tours, data50, Worst tour time: 1.68
# 
#     


    # scenarios = [1,2,3,4,5,6]
    scenarios = [5]
    secBreak  = 1.6 # seconds:  Shims worst tour time: 11s / 7 nodes = 1.6s per node
    # secBreak = 5.0 # parallel ACO

    shimsThreshold = 0.25 # best volume threshold
    acoThreshold   = 0.95

    # method    = "Shims"
    method    = "mpShims"
    # method    = "GRB"

    # method    = "ACO"
    # method    = "mpACO"

    # surplus   = "data20"
    surplus   = "data50"
    # surplus   = "data100"

    dists = common.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    # import sys
    # method  =  f"{sys.argv[1]}"
    # surplus =  f"{sys.argv[2]}"

    for scenario in scenarios:

        instances = [1,2,3,4,5,6,7]
        # instances = [1]

        cfg = common.Config(scenario)
        
        for i, cols in enumerate(dists):
            for j, dist in enumerate(cols):
                costs[i][j] = cfg.kmCost*dist

        pallets = common.loadPallets(cfg)

        # pallets capacity
        cfg.weiCap = 0
        cfg.volCap = 0
        for p in pallets:
            cfg.weiCap += p.W
            cfg.volCap += p.V

        # smaller aircrafts may have a payload lower than pallets capacity
        if cfg.weiCap > cfg.payload:
            cfg.weiCap = cfg.payload

        perc = 1.0
        if cfg.numNodes > 3:
            perc = 0.25

        instanceTime = 0.
        instanceSC   = 0.
        worstTime    = 0
        for instance in instances:

            bestSC = 0. # maximum score/cost relation
            tours = common.getTours(cfg.numNodes-1, costs, perc)

            # selects the best tour
            searchTime = 0
            for pi, tour in enumerate(tours):

                # if pi == 1:

                tour.elapsed = 0
                tour.score   = 0.0

                solveTour(scenario, instance, pi, tour, method, pallets, cfg, secBreak, surplus)

                searchTime += tour.elapsed

                tourSC = tour.score / tour.cost

                # best tour parameters
                if tourSC > bestSC:
                    bestSC = tourSC

                if tour.elapsed > worstTime:
                    worstTime = tour.elapsed
            
            instanceTime += searchTime
            instanceSC   += bestSC

        numInst = float(len(instances))

        avgTime = math.ceil(instanceTime/numInst)

        # instances average
        writeAvgResults(method, scenario, f"{instanceSC/numInst:.2f}\t {avgTime:.0f}\n", surplus)

        print(f"{method}_{scenario}, {instanceSC/numInst:.2f}, {avgTime:.0f}, {len(tours)} tours, {surplus}, Worst tour time: {worstTime:.2f}")

        print(f"secBreak: {secBreak}")

        if "ACO" in method:
             print(f"acoThreshold: {acoThreshold:.2f}")

        if "Shims" in method:
             print(f"shimsThreshold: {shimsThreshold:.2f}")