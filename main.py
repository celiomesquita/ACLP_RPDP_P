import common
import numpy as np
import time
import multiprocessing as mp
import os
import mpShims
import mpACO
import optcgcons
import mipGRB

def solveTour(scenario, inst, pi, tour, method, pallets, cfg, secBreak, surplus):
    """
    Solves one tour
    """
    print(f"----- Tour {pi},", end='')

    for node in tour.nodes:
        print(f" {node.ICAO}", end='')
    print()



    base = len(tour.nodes)-1

    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

        # solution global torque to be shared and changed by all pallets concurrently
        solTorque = mp.Value('d', 0.0) # a multiprocessing double type variable
        
        # initialize- the accumulated values
        sNodeAccum = 0.
        wNodeAccum = 0.
        vNodeAccum = 0.

        # a matrix for all consolidated in the tour
        consol = [
                    [ common.Item(-1, -2, 0, 0, 0., -1, -1)
                    for _ in tour.nodes ]
                    for _ in pallets # a consolidated for each pallet
                ]            

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # load items parameters from this node and problem instance, that go to unnatended
        items = common.loadNodeItems(scenario, inst, node, unattended, surplus)
        numItems = len(items)
        # print(f"number of items: {numItems}")

        if k > 0 and k < base: # not in the base

            # load consolidated generated in the previous node
            prevNode = tour.nodes[k-1]

            cons = common.loadNodeCons(surplus, scenario, inst, pi, prevNode, numItems )

            # consolidated contents not destined to this point are kept on board ...
            kept = []
            cid = N
            for c in cons:
                if c.To in unattended:
                    c.ID = cid
                    kept.append(c) #... and included in the items set
                    cid += 1
            print(f"({len(kept)} consolidated kept on board)")

            # Optimize consolidated positions to minimize CG deviation.
            # Pallets destinations are also set, according to kept on board in new positions
            # Kept P is not -2 anymore, but the pallet ID.
            optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)

            # N: number of items to embark
            # put the consolidated on their assgined pallets (OptCGCons)
            for c in kept:
                for i, p in enumerate(pallets):
                    if c.P == p.ID:
                        pallets[i].putConsol( c, solTorque)

                        # update the consolidated of the current node "k"
                        consol[i][k].ID  = j+N
                        consol[i][k].Frm = node.ID
                        consol[i][k].To  = pallets[i].Dests[k]
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
            mpShims.Solve(pallets, items, cfg, k, 0.95, secBreak, "p", solTorque, solDict, itemsDict)

        if method == "Shims":            
            mpShims.Solve(pallets, items, cfg, k, 0.95, secBreak, "s", solTorque, solDict, itemsDict)         

        if method == "mpACO":       
            mpACO.Solve(pallets,   items, cfg, k, 0.85, secBreak, "p", solTorque, solDict, itemsDict) 

        if method == "ACO":       
            mpACO.Solve(pallets,   items, cfg, k, 0.85, secBreak, "s", solTorque, solDict, itemsDict) 

        if method == "GRB":       
            mipGRB.Solve(pallets,  items, cfg, k,       secBreak,      solTorque, solDict, itemsDict) 

        nodeElapsed = time.perf_counter() - startNodeTime

        tour.elapsed += nodeElapsed

        Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)

        for i, row in enumerate(Y):
            for j, X_ij in enumerate(row):
                if X_ij:    
                    consol[i][k].W += items[j].W
                    consol[i][k].V += items[j].V
                    consol[i][k].S += items[j].S

                    sNodeAccum += float(items[j].S)
                    wNodeAccum += float(items[j].W)
                    vNodeAccum += float(items[j].V)

                    tour.score += items[j].S  

        epsilom = solTorque.value/cfg.maxTorque
        tour.cost *= ( 1.0 + abs(epsilom)/20.0 )

        consNodeT = [None for _ in pallets]        
        for i, p in enumerate(pallets):
            consNodeT[i] = consol[i][k]

        print(f"----- node {node.ICAO},", end='')
        print(f" score {tour.score:.0f}, cost {tour.cost:.0f} -----\n")

        # write consolidated contents from this node in file
        common.writeNodeCons(scenario, instance, consNodeT, pi, node, surplus)
            
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

# Shims_6,   0.287, 336.990,  123 tours, data50, Worst tour time: 3.2
# mpShims_6, 0.287, 333.710,  123 tours, data50, Worst tour time: 3.0
# ACO_6,     0.212, 4804.305, 123 tours, data50, Worst tour time: 42.18

"""
466 items  18 pallets
Greedy initial score 46657.0
^CTraceback (most recent call last):
  File "/usr/lib/python3.10/runpy.py", line 196, in _run_module_as_main
    return _run_code(code, main_globals, None,
  File "/usr/lib/python3.10/runpy.py", line 86, in _run_code
    exec(code, run_globals)
  File "/home/celio/Projects/ACLP_RPDP_P/main.py", line 260, in <module>
    solveTour(scenario, instance, pi, tour, method, pallets, cfg, secBreak, surplus)
  File "/home/celio/Projects/ACLP_RPDP_P/main.py", line 117, in solveTour
    mpACO.Solve(pallets,   items, cfg, k, 0.85, secBreak, "p", solTorque, solDict, itemsDict) 
  File "/home/celio/Projects/ACLP_RPDP_P/mpACO.py", line 262, in Solve
    if accumsP[a].value > bestAntScore:
  File "<string>", line 3, in getvalue
KeyboardInterrupt"""


    # scenarios = [1,2,3,4,5,6]
    scenarios = [6]
    secBreak  = 0.5 # second  Shims worst tour time: 3.2 s -> 3.2/7 = 0.46

    # method    = "Shims"
    # method    = "mpShims"
    # method    = "ACO"
    method    = "mpACO"
    # method    = "GRB"

    # surplus   = "data20"
    surplus   = "data50"
    # surplus   = "data100"

    dists = common.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    # import sys
    # method  =  f"{sys.argv[1]}"
    # surplus =  f"{sys.argv[2]}"

    for scenario in scenarios:

        if scenario == 1:
            instances = [1,2,3,4,5,6,7]
            # instances = [1]
        if scenario == 2:
            instances = [1,2,3,4,5,6,7]
            # instances = [1]
        if scenario == 3:
            instances = [1,2,3,4,5,6,7]
        if scenario == 4:
            instances = [1,2,3,4,5,6,7]
        if scenario == 5:
            instances = [1,2,3,4,5]
        if scenario == 6:
            instances = [1,2,3,4,5,6,7]                                        

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
        if cfg.numNodes >= 3:
            perc = 0.25

        tours = common.getTours(cfg.numNodes-1, costs, perc)

        instanceTime = 0.
        instanceSC   = 0.
        worstTime    = 0
        for instance in instances:

            bestSC = 0. # maximum score/cost relation

            # selects the best tour
            searchTime = 0
            for pi, tour in enumerate(tours):

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

        # instances average
        writeAvgResults(method, scenario, f"{instanceSC/numInst:.3f}\t{instanceTime/numInst:.3f}\n", surplus)

        print(f"{method}_{scenario}, {instanceSC/numInst:.3f}, {instanceTime/numInst:.3f}, {len(tours)} tours, {surplus}, Worst tour time: {worstTime:.2f}")
