import methods as mno
import numpy as np
import time
import math

import shims_p
import shims
import aco
import aco_p
import greedy

import sys
method   = f"{sys.argv[1]}"
mno.DATA = f"{sys.argv[2]}"

# scenarios = [1,2,3,4,5,6]
scenarios = [1]
bests = []

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
        instances = [1,2,3] 

    limit    = 0.92 # Shims and ACO
    secBreak = 0.7 # seconds

    cfg = mno.Config(scenario)                                      

    dists = mno.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value

    pallets = mno.loadPallets(cfg)

    # pallets capacity
    cfg.weiCap = 0
    cfg.volCap = 0
    for p in pallets:
        cfg.weiCap += p.W
        cfg.volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if cfg.weiCap > cfg.payload:
        cfg.weiCap = cfg.payload   

    totElapsed = 0
    sumScores = 0

    numProcs = [1,4,8,12,16]

    if method == "Shims" or method == "ACO" or method == "Greedy":
        numProcs = [1]

    runtimes = [0 for _ in numProcs]
    scores = [0 for _ in numProcs]

    for inst in instances:    

        for ix, procs in enumerate(numProcs):

            tours = mno.getTours(cfg.numNodes-1, costs, 0.25)

            pi = 0 # the first, not necessarily the best

            tour = tours[pi]

            k = 0 # the base

            # L_k destination nodes set
            unattended = [n.ID for n in tour.nodes[k+1:]]

            node = tour.nodes[k]
            print(node.ICAO)

            # load items parameters from this node and problem instance, that go to unnatended
            items = mno.loadNodeItems(scenario, inst, node, unattended)

            numItems = len(items)

            print(f"{numItems} items and {procs} processes")

            mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

            print("Dests: ",end="")
            for p in pallets:
                print(f"{mno.CITIES[p.Dests[k]]} ", end='')
            print()

            E = []
            bestLim = 0.

            startNodeTime = time.perf_counter()

            if method == "Greedy":
                E = greedy.Solve(pallets, items, cfg, k)

            if method == "Shims_p":
                E = shims_p.Solve(pallets, items, cfg, k, limit, procs)           

            if method == "Shims":
                E = shims.Solve(pallets, items, cfg, k, limit)

            if method == "ACO":
                E =   aco.Solve( pallets, items, startNodeTime, cfg, k, limit, secBreak)

            if method == "ACO_p":
                E = aco_p.Solve( pallets, items, startNodeTime, cfg, k, limit, procs, secBreak)

            elapsed = time.perf_counter() - startNodeTime

            consJK = [
                        [ mno.Item(-1, -2, 0, 0, 0., -1, -1)
                        for _ in tour.nodes ]
                        for _ in pallets # a consolidated for each pallet
                    ] 

            # print the solution for this node
            if len(E) > 0:

                consNodeT = [None for _ in pallets]

                pallets.sort(key=lambda x: x.ID)  

                for j, p in enumerate(pallets):

                    consJK[j][k].ID  = j+numItems
                    consJK[j][k].Frm = node.ID
                    consJK[j][k].To  = p.Dests[k]

                    itemsCount = [0 for _ in np.arange(numItems)]

                    for i in np.arange(numItems):

                        if E[j][i] == 1:

                            itemsCount[i] += 1

                            consJK[j][k].W += items[i].W
                            consJK[j][k].V += items[i].V
                            consJK[j][k].S += items[i].S

                    consNodeT[j] = consJK[j][k]

                sNodeAccum = 0.
                wNodeAccum = 0.
                vNodeAccum = 0.
                tau = 0.
                sol = ""

                for i, p in enumerate(pallets):
                    sNodeAccum += float(consJK[i][k].S)
                    wNodeAccum += float(consJK[i][k].W)
                    vNodeAccum += float(consJK[i][k].V)
                    tau        += float(consJK[i][k].W) * pallets[i].D

                epsilom = tau/cfg.maxTorque

                # greedyScore = 50842.0 # data50

                sol += f"Score: {sNodeAccum}\t"
                sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
                sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
                sol += f"Torque: {epsilom:.2f}\n"
                sol += f"Elapsed: {elapsed:.2f}\n"
                # sol += f"Ratio: {sNodeAccum/greedyScore:.3f}\n"

                state = "Feasible"
                for n in itemsCount:
                    if n > 1:
                        state = "Unfeasible"

                if wNodeAccum/cfg.weiCap > 1.0:
                    state = "Unfeasible"

                if vNodeAccum/cfg.volCap > 1.0:
                    state = "Unfeasible"

                if abs(epsilom) > 1.0:
                    state = "Unfeasible"

                sol += f"State: {state}\n"

                print(sol)

                totElapsed += elapsed
                sumScores  += sNodeAccum

                if method == "Shims_p" or method == "ACO_p":
                    runtimes[ix]  += elapsed
                    scores[ix] += sNodeAccum

    if len(numProcs) > 1:

        text = "np,time,score\n"
        for i, _ in enumerate(numProcs):

            scores[i] /= len(instances)
            runtimes[i] /= len(instances)

            text += f"{numProcs[i]},{runtimes[i]:.2f},{scores[i]:.0f}\n"          

        fname = f"./latex/csv/{method}{scenario}{mno.DATA}.csv"

        writer = open(fname,'w+') # + creates the file, if not exists

        try:
            writer.write(text)
        finally:
            writer.close() 

        print(text)

        bestNumProcs = -1
        minDist = 999999.9
        maxScore = max(scores)
        minScore = min(scores)      
        maxTime = max(runtimes)
        minTime = min(runtimes)  

        for i, _ in enumerate(numProcs):

            if (maxScore - minScore) and (maxTime  - minTime) > 0:

                yLeg = (maxScore - scores[i])/(maxScore - minScore)
                xLeg = (runtimes[i] - minTime)  /(maxTime  - minTime)

                distance = math.sqrt(xLeg**2 + yLeg**2)

                if minDist > distance:
                    minDist = distance
                    bestNumProcs = numProcs[i]
            
            # print(f"{numProcs[i]}\t{distance:.2f}\t{xLeg:.2f}\t{yLeg:.2f}")

        # print(f"The best number of processes is: {bestNumProcs}")

        bests.append(bestNumProcs)

    print(f"{bests}")