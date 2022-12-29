import common
import numpy as np
import time
import multiprocessing as mp
import os
import shims_mp
import aco_mp
import optcgcons

solTorque = mp.Value('d') # solution global torque to be shared and changed by all pallets concurrently
solTorque.value = 0.0

def solveTour(scenario, inst, pi, tour, method, pallets, cfg, numProcs, secBreak, surplus):
    """
    Solves one tour
    """
    tour.elapsed = 0
    broke = 0

    # first line for result file
    sol = f"Tour {pi}, with {cfg.numNodes} nodes and the {cfg.size} aircraft\n"

    consJK = [
                [ common.Item(-1, -2, 0, 0, 0., -1, -1)
                  for _ in tour.nodes ]
                for _ in pallets
             ]          

    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

            
        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # load items parameters from this node and problem instance, that go to unnatended
        items = common.loadNodeItems(scenario, inst, node, unattended, surplus)
        numItems = len(items)
        # print(f"number of items: {numItems}")

        # load consolidated generated in the previous node
        prevNode = tour.nodes[k-1]

        # numItems = first cons ID
        cons = common.loadNodeCons(surplus, scenario, inst, pi, prevNode, numItems )

        # if prevNode.ID < len(common.CITIES):
        #     print(f"\n-----Loaded in {common.CITIES[prevNode.ID]} -----")
        #     print("ID\tP\tW\tS\tV\tFROM\tTO")
        #     for c in cons:
        #         print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
        # print(f"({numItems} items to embark)")


        # consolidated contents not destined to this point are kept on board ...
        kept = []
        for c in cons:
            if c.To in unattended:
                c.ID = numItems
                kept.append(c) #... and included in the items set
                numItems += 1

        # print(f"\n----- Kept on board at {common.CITIES[node.ID]} -----")        
        # print("ID\tP\tW\tS\tV\tFROM\tTO")
        # for c in kept:
        #     print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
        # print(f"Kept positions to be defined: ({numItems} items to embark)\n")

        # optimize consolidated positions to minimize CG deviation
        optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)
        # pallets destinations are also set, according to kept on board in new positions

        # Kept P is not -2 anymore, but the pallet ID.
        
        # print("ID\tP\tW\tS\tV\tFROM\tTO")
        for c in kept:
            # consolidated are appended to the items set
            items.append(c)
            # print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
        # print(f"Kept positions defined ({numItems} items to embark)\n")

        # print("ID\tDest\tPCW\tPCV\tPCS")
        # for p in pallets:
        #     print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
        # print("Pallets destinations to be defined.\n")

        # set pallets destinations with items and consolidated to be delivered
        common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

        # print("ID\tDest\tPCW\tPCV\tPCS")
        # for p in pallets:
        #     print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
        # print("Pallets destinations defined.\n")


        # to control solution items
        solItems = mp.Array('i', range(numItems))
        for j, _ in enumerate(solItems):
            solItems[j] = -1 # not alocated to any pallet

        # put the kept on board in solution
        for c in kept:
            solItems[c.ID] = c.P

        # update pallets current parameters and solution torque
        for i, p in enumerate(pallets):
            for c in kept:
                if c.P == p.ID:
                    pallets[i].PCW += c.W
                    pallets[i].PCV += c.V
                    pallets[i].PCS += c.S
                    solTorque.value += float(c.W) * float(p.D)

        #     print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
        # print(f"Pallets are now with current values defined. Torque: {solTorque.value/cfg.maxTorque:.2f}\n")
        
        E = []
        startNodeTime = time()

        limit  = 0.95 # Shims and ACO

        if method == "Shims_mp":
            E = shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque, solItems)

        if method == "Shims":            
            E = shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "s", solTorque, solItems)         

        if method == "ACO_mp":       
            E =   aco_mp.Solve(pallets, items, cfg, k, limit, secBreak, "a", solTorque, solItems) 

        nodeElapsed = time() - startNodeTime

        tour.elapsed += nodeElapsed

        # print the solution for this node
        if len(E) > 0:

            consNodeT = [None for _ in pallets]

            itemsCount = [0 for _ in np.arange(numItems)]

            pallets.sort(key=lambda x: x.ID)  

            for j, p in enumerate(pallets):

                consJK[j][k].ID  = j+numItems
                consJK[j][k].Frm = node.ID
                consJK[j][k].To  = p.Dests[k]


                for i in np.arange(numItems):

                    if E[j][i] == 1:

                        itemsCount[i] += 1

                        consJK[j][k].W += items[i].W
                        consJK[j][k].V += items[i].V
                        consJK[j][k].S += items[i].S

                consNodeT[j] = consJK[j][k]

            state = "Feasible"
            for n in itemsCount:
                if n > 1:
                    state = "Unfeasible"
                    break

            print(f"----- TOUR {pi} {node.ICAO} END ----- {state}\n")

            # write consolidated contents from this node in file
            common.writeNodeCons(scenario, instance, consNodeT, pi, node)

    common.writeTourSol(method, scenario, instance, pi, tour, cfg, pallets, consJK, False) # False -  does not generate latex solution table
            
    return broke

# end of solveTour 

def writeAvgResults(method, scenario, line):

    dirname = f"./results/{common.DATA}/{method}_{scenario}"
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


    # scenarios = [1,2,3,4,5,6]
    scenarios = [1]
    secBreak  = 0.7
    numProcs  = 10

    dists = common.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    import sys
    method  =  f"{sys.argv[1]}"
    surplus =  f"{sys.argv[2]}"

    for scenario in scenarios:

        if scenario == 1:
            # instances = [1,2,3,4,5,6,7]
            instances = [1]
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
            instances = [1,2,3]                                        

        cfg = common.Config(scenario)
        
        for i, cols in enumerate(dists):
            for j, value in enumerate(cols):
                costs[i][j] = cfg.kmCost*value

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

        tours = common.getTours(cfg.numNodes-1, costs, 0.25)

        broke         = 0
        avgInstTime   = 0.
        avgInstNumOpt = 0.
        avgInstSC     = 0.
        # worstDuration = 0
        for instance in instances:

            # print(f"-> method: {method} scenario: {scenario} instance: {instance} | Tours {len(tours)}")

            bestSC = 0. # maximum score/cost relation

            # selects the best tour
            searchTime = 0
            for pi, tour in enumerate(tours):

                broke = solveTour(scenario, instance, pi, tour, method, pallets, cfg, numProcs, secBreak, surplus) # writeTourSol is True or False

                searchTime += tour.elapsed

                # if tour.elapsed > worstDuration :
                    # worstDuration = tour.elapsed
                    
                curSC = tour.score / tour.cost

                # best tour parameters
                if curSC > bestSC:
                    bestSC = curSC
            
            avgInstTime   += searchTime
            avgInstSC     += bestSC

        numInst = float(len(instances))

        timeString = common.getTimeString(avgInstTime, numInst, True)

        avgInstSC /= numInst

        # instances average
        writeAvgResults(method, scenario, f"{avgInstSC:.2f}\t{timeString}\n")

        # print(f"{method}\t{scenario}\t{avgInstSC:.2f}\t{timeString}\t{len(tours)} tours")
