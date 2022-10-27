import numpy as np
from tabulate import tabulate
import time
import os
# local packages
import methods as mcuda
import optcgcons
import shims
import greedy

def solveTour(scenario, instance, pi, tour, method, pallets, cfg):
    """
    Solves one tour
    """
    tour.elapsed = 0
    broke = 0

    # first line for result file
    sol = f"Tour {pi}, with {cfg.numNodes} nodes and the {cfg.size} aircraft\n"

    consJK = [
                [ mcuda.Item(-1, -2, 0, 0, 0., -1, -1)
                  for _ in tour.nodes ]
                for _ in pallets
             ]          

    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

        # L_k destination nodes set
        unattended = []
        for n in tour.nodes[k+1:]:
            unattended.append(n.ID)

        # all nodes are attended, this node is the base, stops
        if len(unattended) == 0:
            break

        # load items parameters from this node and problem instance, that go to unnatended
        items = mcuda.loadNodeItems(scenario, instance, node, unattended)

        numItems = len(items)
        numKept = 0

        print(f"-> Tour:{pi} sequence:{k} city:{node.ICAO} items:{numItems}")

        cons = []

        # reset pallets destinations
        for i, _ in enumerate(pallets):
            pallets[i].Dests = [-1]*cfg.numNodes

        if k > 0: # not in the base

            # load consolidated generated in the previous node
            prevNode = tour.nodes[k-1]
            cons = mcuda.loadNodeCons( scenario, instance, pi, prevNode, numItems ) # numItems = first cons ID

            print(f"\n-----Loaded from tour {pi} {mcuda.CITIES[prevNode.ID]} -----")
            print("P\tW\tS\tV\tFROM\tTO")
            for c in cons:
                print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                        c.P, c.W, c.S, c.V, mcuda.CITIES[c.Frm], mcuda.CITIES[c.To]))

            # consolidated contents not destined to this point are kept on board
            kept = []
            for c in cons:
                if c.To in unattended:
                    kept.append(c)
                    
            # optimize consolidated positions to minimize CG deviation
            # it is not necessary with the MIP solver
            if len(kept) > 0 and method != "GRB":
                print("\n----- optimize consolidated positions -----")
                optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)

            print(f"\n-----Consolidated contents from tour {pi}, {mcuda.CITIES[prevNode.ID]} kept on board -----")

            print("P\tW\tS\tV\tFROM\tTO")
            for c in kept:
                items.append(c)
                numItems += 1
                numKept  += 1
                print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                    c.P,c.W, c.S, c.V, mcuda.CITIES[c.Frm], mcuda.CITIES[c.To]))

        # set pallets destinations with items and consolidated to be delivered
        print("\n----- setPalletsDestinations, Carlos' version -----")
        mcuda.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)                

        print("Dests: ",end="")
        for p in pallets:
            print(f"{mcuda.CITIES[p.Dests[k]]} ", end='')
        print()

        print(f"-> {numItems} items with {numKept} kept on board in {node.ICAO}")

        E = []
        startNodeTime = time.perf_counter()

        if method == "Shims":
            E = shims.Solve(pallets, items, cfg, k)

        if method == "Greedy":
            E = greedy.Solve(pallets, items, cfg, k)  

        nodeElapsed = time.perf_counter() - startNodeTime

        tour.elapsed += nodeElapsed

        # print the solution for this node
        if len(E) > 0:

            consNodeT = [None for _ in pallets]

            pallets.sort(key=lambda x: x.ID)  

            for j, p in enumerate(pallets):

                consJK[j][k].ID  = j+numItems
                consJK[j][k].Frm = node.ID
                consJK[j][k].To  = p.Dests[k]

                for i in np.arange(numItems):

                    if E[j][i] == 1:

                        consJK[j][k].W += items[i].W
                        consJK[j][k].V += items[i].V
                        consJK[j][k].S += items[i].S

                consNodeT[j] = consJK[j][k]

            print(f"----- TOUR {pi} {node.ICAO} END -----\n")

            # write consolidated contents from this node in file
            mcuda.writeNodeCons(scenario, instance, consNodeT, pi, node)

    mcuda.writeTourSol(method, scenario, instance, pi, tour, cfg, pallets, consJK, False) # False -  does not generate latex table
            
    return broke

# end of solveTour 

def writeAvgResults(method, scenario, line):

    dirname = f"./results/{mcuda.DATA}/{method}_{scenario}"
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

    # from numba import cuda
    # print(cuda.gpus)    

    import sys

    scenario  = int(sys.argv[1])
    method    =  f"{sys.argv[2]}"
    # mcuda.NCPU = int(sys.argv[3])

    # clear cache
    # find . | grep -E "(__pycache__|\.pyc|\.pyo$)" | xargs rm -rf

    mcuda.SEC_BREAK = 0.7
    # mcuda.SEC_BREAK = 10

    mcuda.DATA = "data20"
    # mcuda.DATA = "data50"
    # mcuda.DATA = "data100"
  

    # scenario = 1

    if scenario == 1:
        # instances = [1,2,3,4,5,6,7]
        instances = [1]
    if scenario == 2:
        instances = [1,2,3,4,5,6,7]
    if scenario == 3:
        instances = [1,2,3,4,5,6,7]
    if scenario == 4:
        instances = [1,2,3,4,5,6,7]
    if scenario == 5:
        instances = [1,2,3,4,5]
    if scenario == 6:
        instances = [1,2,3]                                        

    dists = mcuda.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    cfg = mcuda.Config(scenario)
    
    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value

    # for method in ["ACO","ACO_mp","Greedy","Shims","Shims_mp"]:
    # for method in ["Shims_mp"]:

    pallets = mcuda.loadPallets(cfg)

    # pallets capacity
    cfg.weiCap = 0
    cfg.volCap = 0
    for p in pallets:
        cfg.weiCap += p.W
        cfg.volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if cfg.weiCap > cfg.payload:
        cfg.weiCap = cfg.payload

    tours = mcuda.getTours(cfg.numNodes, costs, 0.05)

    broke = 0
    avgInstTime = 0.
    avgInstNumOpt = 0.
    avgInstSC = 0.
    # worstDuration = 0
    for instance in instances:

        print(f"-> method: {method} scenario: {scenario} instance: {instance}")

        bestSC = 0. # maximum score/cost relation

        # selects the best tour
        searchTime = 0
        for pi, tour in enumerate(tours):

            broke = solveTour(scenario, instance, pi, tour, method, pallets, cfg) # writeTourSol is True or False

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

    timeString = mcuda.getTimeString(avgInstTime, numInst)

    avgInstSC /= numInst

    latex = f"{avgInstSC:.3f}   &   {timeString}\n"

    # instances average
    writeAvgResults(method, scenario, latex)

    print(f"{method}\t{scenario}\t{latex}")

        