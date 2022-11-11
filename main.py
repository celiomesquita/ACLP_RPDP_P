import numpy as np
from tabulate import tabulate
from time import time
import math
from os import path
import os
# local packages
import methods
import optcgcons
import shims
import shims_p
import aco
import aco_p
import greedy


def solveTour(scenario, instance, pi, tour, method, pallets, cfg, numCPU, secBreak):
    """
    Solves one tour
    """
    tour.elapsed = 0
    broke = 0

    # first line for result file
    sol = f"Tour {pi}, with {cfg.numNodes} nodes and the {cfg.size} aircraft\n"

    consJK = [
                [ methods.Item(-1, -2, 0, 0, 0., -1, -1)
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
        items = methods.loadNodeItems(scenario, instance, node, unattended)

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
            cons = methods.loadNodeCons( scenario, instance, pi, prevNode, numItems ) # numItems = first cons ID

            if prevNode.ID < len(methods.CITIES):
                print(f"\n-----Loaded from tour {pi} {methods.CITIES[prevNode.ID]} -----")
                print("P\tW\tS\tV\tFROM\tTO")
                for c in cons:
                    print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                            c.P, c.W, c.S, c.V, methods.CITIES[c.Frm], methods.CITIES[c.To]))

            # consolidated contents not destined to this point are kept on board
            kept = []
            for c in cons:
                if c.To in unattended:
                    kept.append(c)
                    
            # optimize consolidated positions to minimize CG deviation
            if len(kept) > 0 and method != "GRB":
                print("\n----- optimize consolidated positions -----")
                optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "CBC", k)

            if prevNode.ID < len(methods.CITIES):
                print(f"\n-----Consolidated contents from tour {pi}, {methods.CITIES[prevNode.ID]} kept on board -----")

            print("P\tW\tS\tV\tFROM\tTO")
            for c in kept:
                items.append(c)
                numItems += 1
                numKept  += 1
                if prevNode.ID < len(methods.CITIES):
                    print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                        c.P,c.W, c.S, c.V, methods.CITIES[c.Frm], methods.CITIES[c.To]))

        # set pallets destinations with items and consolidated to be delivered
        print("\n----- setPalletsDestinations, Carlos' version -----")
        methods.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)                

        print("Dests: ",end="")
        for p in pallets:
            print(f"{methods.CITIES[p.Dests[k]]} ", end='')
        print()

        print(f"-> {numItems} items with {numKept} kept on board in {node.ICAO}")

        E = []
        opt = 0
        startNodeTime = time()

        minLim = 0.75 # Shims_p
        limit  = 0.95 # Shims and ACO

        if method == "Greedy":
            E = greedy.Solve(pallets, items, cfg, k)

        if method == "Shims_p":
            E = shims_p.Solve(pallets, items, cfg, k, minLim, numCPU)

        if method == "Shims":
            E = shims.Solve(pallets, items, cfg, k, limit)

        if method == "ACO":
            E =   aco.Solve( pallets, items, startNodeTime, cfg, k, limit, secBreak)

        if method == "ACO_p":
            E = aco_p.Solve( pallets, items, startNodeTime, cfg, k, limit, numCPU, secBreak) 

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
            methods.writeNodeCons(scenario, instance, consNodeT, pi, node)


           

    methods.writeTourSol(method, scenario, instance, pi, tour, cfg, pallets, consJK, False) # False -  does not generate latex solution table
            
    return broke

# end of solveTour 

def writeAvgResults(method, scenario, line):

    dirname = f"./results/{methods.DATA}/{method}_{scenario}"
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

    import sys
    scenario     =   int(sys.argv[1])
    method       =    f"{sys.argv[2]}"
    numCPU       =   int(sys.argv[3])
    methods.DATA =    f"{sys.argv[4]}"

    # clear cache
    # find . | grep -E "(__pycache__|\.pyc|\.pyo$)" | xargs rm -rf

    secBreak = 0.7

    # methods.DATA = "data20"
    # methods.DATA = "data50"
    # methods.DATA = "data100"

    # method = "Greedy"

    # scenario = 1

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
        instances = [1,2,3]                                        

    dists = methods.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    cfg = methods.Config(scenario)
    
    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value


    pallets = methods.loadPallets(cfg)

    # pallets capacity
    cfg.weiCap = 0
    cfg.volCap = 0
    for p in pallets:
        cfg.weiCap += p.W
        cfg.volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if cfg.weiCap > cfg.payload:
        cfg.weiCap = cfg.payload

    tours = methods.getTours(cfg.numNodes, costs, 0.25)

    broke = 0
    avgInstTime = 0.
    avgInstNumOpt = 0.
    avgInstSC = 0.
    # worstDuration = 0
    for instance in instances:

        # print(f"-> method: {method} scenario: {scenario} instance: {instance} | Tours {len(tours)}")

        bestSC = 0. # maximum score/cost relation

        # selects the best tour
        searchTime = 0
        for pi, tour in enumerate(tours):

            broke = solveTour(scenario, instance, pi, tour, method, pallets, cfg, numCPU, secBreak) # writeTourSol is True or False

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

    timeString = methods.getTimeString(avgInstTime, numInst, True)

    avgInstSC /= numInst

    latex = f"{avgInstSC:.2f}   &   {timeString}\n"

    # instances average
    writeAvgResults(method, scenario, latex)

    print(f"{method}\t{scenario}\t{avgInstSC:.2f}\t{timeString}\t{len(tours)} tours")
