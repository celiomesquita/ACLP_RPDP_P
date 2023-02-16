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

def solveTour(scenario, inst, pi, tour, method, pallets, cfg, secBreak, surplus, tipo, numOptDict, rampDistCG, afterDict, beforeDict):
    """
    Solves one tour
    """
    writeConsFile = False

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

    k0 = len(tour.nodes)-1 # the base index on return

    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

        next = tour.nodes[k0]
        if k < k0:
            next = tour.nodes[k+1]

        for i, p in enumerate(pallets):
            pallets[i].reset(cfg.numNodes)

        # solution global torque to be shared and changed by all processes concurrently
        nodeTorque = mp.Value('d', 0.0) # a multiprocessing double type variable
        nodeVol    = 0.0
        
        # initialize- the accumulated values
        if writeConsFile:
            wNodeAccum = 0.
            vNodeAccum = 0.

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # load items parameters from this node and problem instance, that go to unnatended
        items = common.loadNodeItems(scenario, inst, node, unattended, surplus)

        if k > 0 and k < k0: # not in the base

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

                nodeTorque.value += optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, k)

                # N: number of items to embark
                for c in kept:
                    for i, p in enumerate(pallets):
                        if c.P == p.ID:
                            pallets[i].putConsol(c)

                            # update the consolidated of the current node "k"
                            consol[i][k].ID  = j+N
                            consol[i][k].Frm = node.ID
                            consol[i][k].To  = pallets[i].Dest[k]
                            consol[i][k].W  += c.W
                            consol[i][k].V  += c.V
                            consol[i][k].S  += c.S

                            # update the accumulated values
                            if writeConsFile:
                                wNodeAccum += c.W
                                vNodeAccum += c.V

                            tour.score += c.S
                            nodeVol    += c.V


        # set pallets destinations with items and consolidated to be delivered
        if k < k0: # except when the current node is the base on returning
            common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)
        else:
            return # skip solving because it's the base on returning

        startNodeTime = time.perf_counter()

        # to control solution items
        M = len(pallets)
        N = len(items)
        # mp.Array to be shared by multiprocess jobs
        solMatrix = mp.Array('i', [0 for _ in np.arange(N*M)] )
        mpItems   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

        # I use dict to pass by reference
        solDict   = dict(solMatrix=solMatrix)
        itemsDict = dict(mpItems=mpItems)
        modStatus = 0

        if method == "mpShims":
            mpShims.Solve(pallets, items, cfg, k, volThreshold, secBreak, "p", nodeTorque, solDict, itemsDict, tipo)

        if method == "Shims":            
            mpShims.Solve(pallets, items, cfg, k, volThreshold, secBreak, "s", nodeTorque, solDict, itemsDict, tipo)         

        if method == "mpACO":       
            mpACO.Solve(  pallets, items, cfg, k, volThreshold, secBreak, "p", nodeTorque, solDict, itemsDict) 

        if method == "ACO":       
            mpACO.Solve(  pallets, items, cfg, k, volThreshold, secBreak, "s", nodeTorque, solDict, itemsDict) 

        if method == "GRB":
            modStatus = mipGRB.Solve( pallets, items, cfg, k,               secBreak,      nodeTorque, solDict, itemsDict) 

        if modStatus == 2: # 2: optimal
            numOptDict["numOpt"] += 1


        for p in pallets:
            if p.Dest[k] == next.ID:
                beforeDict['Before'] += rampDistCG - p.D # distance from the pallet to the ramp door

        optcgcons.minRampDist(pallets, k, tour, rampDistCG, cfg, nodeTorque)

        for p in pallets:
            if p.Dest[k] == next.ID:
                afterDict['After'] += rampDistCG - p.D # distance from the pallet to the ramp door


        nodeElapsed = time.perf_counter() - startNodeTime

        tour.elapsed += nodeElapsed

        Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)
       
        nodeScore = 0
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
                    if writeConsFile:
                        wNodeAccum += float(items[j].W)
                        vNodeAccum += float(items[j].V)

                    # tour.score += items[j].S
                    nodeScore  += items[j].S
                    nodeVol    += items[j].V

        nodeVol /= cfg.volCap

        epsilon = nodeTorque.value/cfg.maxTorque

        tour.score += nodeScore

        tour.AvgVol    += nodeVol
        tour.AvgTorque += epsilon

        print(f"----- node {node.ICAO},", end='')
        print(f" score {tour.score:.0f}, cost {tour.cost:.0f}, vol {nodeVol:.2f}, torque {epsilon:.2f} -----")

        if writeConsFile:

            consNodeT = [None for _ in pallets]        
            for i, p in enumerate(pallets):
                consNodeT[i] = consol[i][k]

            vol = vNodeAccum/cfg.volCap
            wei = wNodeAccum/cfg.weiCap

            #write consolidated contents from this node in file
            common.writeNodeCons(scenario, instance, consNodeT, pi, node, surplus, epsilon, wei, vol)


# end of solveTour 

def writeAvgResults(method, scenario, line, surplus):

    dirname = f"./results/{surplus}/"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass  

    fname = f"{dirname}/{method}_{scenario}.avg"

    writer = open(fname,'w+') # + creates the file, if not exists

    try:
        writer.write(line)
    finally:
        writer.close()  

if __name__ == "__main__":

    # import sys
    # method  =  f"{sys.argv[1]}"
    # surplus =  f"{sys.argv[2]}"

    dists = common.loadDistances()
    costs = [[0.0 for _ in dists] for _ in dists]

    secBreak     = 1.8 # second
    volThreshold = 0.92 # 0.92 best for scenario 1

    scenarios = [1,2,3,4,5,6]
    # scenarios = [2]

    # surplus   = "data20"
    # surplus   = "data50"
    surplus   = "data100"

    # methods = ["Shims","mpShims","GRB"]
    
    methods = ["Shims"]
    # methods = ["mpShims"]
    # tipo = "KP"
    tipo = "FFD"

    # methods = ["GRB"]

    for method in methods:

        for scenario in scenarios:

            instances = [1,2,3,4,5,6,7]
            # instances = [1]

            cfg = common.Config(scenario)
            
            for i, cols in enumerate(dists):
                for j, dist in enumerate(cols):
                    costs[i][j] = cfg.kmCost*dist

            pallets, rampDistCG = common.loadPallets(cfg)

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

            numOptDict = {"numOpt":0}
            afterDict  = {"After":0.}
            beforeDict = {"Before":0.}

            for instance in instances:

                bestSC = 0. # maximum score/cost relation
                bestAV = 0.
                bestAT = 0.
                tours = common.getTours(cfg.numNodes-1, costs, perc)

                # selects the best tour
                searchTime = 0
                for pi, tour in enumerate(tours):

                    # if pi == 1:

                    tour.elapsed = 0
                    tour.score   = 0.0
                    tour.AvgVol  = 0.0

                    solveTour(scenario, instance, pi, tour, method, pallets, cfg, secBreak, surplus, tipo, numOptDict, rampDistCG, afterDict, beforeDict)

                    # the tour cost is increased by the average torque deviation, limited to 5%
                    tour.AvgTorque /= cfg.numNodes
                    tour.cost *= ( 1.0 + abs(tour.AvgTorque)/20.0 )

                    searchTime += tour.elapsed

                    tourSC = tour.score / tour.cost

                    tour.AvgVol /= cfg.numNodes

                    # best tour parameters
                    if tourSC > bestSC:
                        bestSC = tourSC
                        bestAV = tour.AvgVol
                        bestAT = tour.AvgTorque

                    if tour.elapsed > worstTime:
                        worstTime = tour.elapsed
                
                instanceTime += searchTime
                instanceSC   += bestSC

            numInst = float(len(instances))

            numOptDict["numOpt"] /= numInst
            numOptDict["numOpt"] /= float(cfg.numNodes)
            numOptDict["numOpt"] /= float(len(tours))

            afterDict["After"] /= numInst
            afterDict["After"] /= float(cfg.numNodes)
            afterDict["After"] /= float(len(tours))

            beforeDict["Before"] /= numInst
            beforeDict["Before"] /= float(cfg.numNodes)
            beforeDict["Before"] /= float(len(tours))

            percent = 0.0
            if beforeDict["Before"] > 0:
                percent = 100.0*(beforeDict["Before"] - afterDict["After"]) / beforeDict["Before"]   

            avgTime = math.ceil(instanceTime/numInst)

            str = f"{instanceSC/numInst:.2f}\t {avgTime:.0f}\t {worstTime:.1f}\t {bestAV:.2f}\t {bestAT:.2f}\t {numOptDict['numOpt']:.1f}\t {beforeDict['Before']:.1f} & {afterDict['After']:.1f} & {percent:.1f}\n"
            # instances average
            writeAvgResults(method, scenario, str, surplus)
            print(f"\n{str}")
            print(f"{surplus}")
            print(f"{len(tours)} tours")
            print(f"secBreak: {secBreak}")
            print(f"volThreshold: {volThreshold:.2f}")
            print(f"Before:\t{beforeDict['Before']:.1f}") 
            print(f"After:\t{afterDict['After']:.1f}")
            print(f"% of optima: {numOptDict['numOpt']:.2f}")
