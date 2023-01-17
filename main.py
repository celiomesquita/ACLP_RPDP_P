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

    for k, node in enumerate(tour.nodes):  # solve each node sequentialy
            
        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # load items parameters from this node and problem instance, that go to unnatended
        items = common.loadNodeItems(scenario, inst, node, unattended, surplus)
        numItems = len(items)
        # print(f"number of items: {numItems}")

        if k > 0: # not in the base

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
            if method != "GRB":
                optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)
            # pallets destinations are also set, according to kept on board in new positions

            # Kept P is not -2 anymore, but the pallet ID.
            
            # print("ID\tP\tW\tS\tV\tFROM\tTO")
            for c in kept:
                # consolidated are appended to the items set
                items.append(c)
            #     print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
            # print(f"Kept positions defined ({numItems} items to embark)\n")

            # print("ID\tDest\tPCW\tPCV\tPCS")
            # for p in pallets:
            #     print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
            # print("Pallets destinations to be defined.\n")

        # set pallets destinations with items and consolidated to be delivered
        if k < len(tour.nodes)-1: # except when the current node is the base on returning
            common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

        # print("ID\tDest\tPCW\tPCV\tPCS")
        # for p in pallets:
        #     print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
        # print("Pallets destinations defined.\n")

        # a multiprocessing array to control solution items
        solItems = mp.Array('i', range(numItems))
        for j, _ in enumerate(solItems):
            solItems[j] = -1 # not alocated to any pallet

        # a multiprocessing variable: solution global torque to be shared and
        # changed by all pallets concurrently when solved by mpShims
        solTorque = mp.Value('d')
        solTorque.value = 0.0

        if k > 0: # not in the base

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

            # print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
        # print(f"Pallets are now with current values defined. Torque: {solTorque.value/cfg.maxTorque:.2f}\n")
        
        startNodeTime = time.perf_counter()

        dictItems = dict(solItems = solItems)
 
        if method == "mpShims":
            mpShims.Solve(pallets, items, cfg, k, 0.95, secBreak, "p", solTorque, dictItems)

        if method == "Shims":            
            mpShims.Solve(pallets, items, cfg, k, 0.95, secBreak, "s", solTorque, dictItems)         

        if method == "mpACO":       
            mpACO.Solve(pallets,   items, cfg, k, 0.85, secBreak, "p", solTorque, dictItems) 

        if method == "ACO":       
            mpACO.Solve(pallets,   items, cfg, k, 0.85, secBreak, "s", solTorque, dictItems) 

        if method == "GRB":       
            mipGRB.Solve(  pallets,   items, cfg, k, secBreak, dictItems) 

        nodeElapsed = time.perf_counter() - startNodeTime

        tour.elapsed += nodeElapsed

        # consolidated matrix: nodes x pallets
        consol = [
                    [ common.Item(-1, -2, 0, 0, 0., -1, -1)
                    for _ in tour.nodes ]
                    for _ in pallets # a consolidated for each pallet
                ] 

        pallets.sort(key=lambda x: x.ID)   

        # Iterate in the solution
        for j, i in enumerate(dictItems["solItems"]):
            if i > -1: # i: pallet index
                p = pallets[i]
                consol[i][k].ID  = j+numItems
                consol[i][k].Frm = node.ID
                consol[i][k].To  = p.Dests[k]

                consol[i][k].W += items[j].W
                consol[i][k].V += items[j].V
                consol[i][k].S += items[j].S

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

    # scenarios = [1,2,3,4,5,6]
    scenarios = [1]
    secBreak  = 0.7 # second

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

        instanceTime   = 0.
        instanceSC     = 0.
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
            
            instanceTime += searchTime
            instanceSC   += bestSC

        numInst = float(len(instances))

        # instances average
        writeAvgResults(method, scenario, f"{instanceSC/numInst:.3f}\t{instanceTime/numInst:.3f}\n", surplus)

        print(f"{method}_{scenario}\t{instanceSC/numInst:.3f}\t{instanceTime/numInst:.3f}\t{len(tours)} tours\t{surplus}")
