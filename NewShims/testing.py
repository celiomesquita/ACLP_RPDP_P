import methods as mno
import numpy as np
import time

import shims_p
import shims

# surplus = "data20"
surplus = "data50"
# surplus = "data100"

method = "Shims_p"
# method = "Shims"

# scenarios = [1,2,3,4,5,6]
scenarios = [1]
bests = []

for scenario in scenarios:

    if scenario == 1:
        # instances = [1,2,3,4,5,6,7]
        instances = [1]
    if scenario == 2:
        # instances = [1,2,3,4,5,6,7]
        instances = [1]
    if scenario == 3:
        # instances = [1,2,3,4,5,6,7]
        instances = [1]
    if scenario == 4:
        instances = [1,2,3,4,5,6,7]
    if scenario == 5:
        instances = [1,2,3,4,5]
    if scenario == 6:
        instances = [1,2,3] 

    # limit    = 0.95 # best
    limit    = 0.05
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

    for inst in instances:    

        tours = mno.getTours(cfg.numNodes-1, costs, 0.25)

        pi = 0 # the first, not necessarily the best

        tour = tours[pi]

        k = 0 # the base

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        node = tour.nodes[k]
        print(node.ICAO)

        # load items parameters from this node and problem instance, that go to unnatended
        items = mno.loadNodeItems(scenario, inst, node, unattended, surplus)

        numItems = len(items)

        print(f"{numItems} items")

        mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

        print("Dests: ",end="")
        for p in pallets:
            print(f"{mno.CITIES[p.Dests[k]]} ", end='')
        print()

        E = []

        startNodeTime = time.perf_counter()

        if method == "Shims_p":
            E = shims_p.Solve(pallets, items, cfg, k, limit, secBreak)

        if method == "Shims":            
            E = shims.Solve(pallets, items, cfg, k, limit, secBreak)         

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

            sol += f"Score: {sNodeAccum}\t"
            sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
            sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
            sol += f"Torque: {epsilom:.2f}\n"
            sol += f"Elapsed: {elapsed:.2f}\n"

            state = "Feasible"
            for n in itemsCount:
                if n > 1:
                    state = "Unfeasible"

            if wNodeAccum/cfg.weiCap > 1.0:
                state = "Weight Unfeasible"

            if vNodeAccum/cfg.volCap > 1.03:
                state = "Volume Unfeasible"

            if abs(epsilom) > 1.0:
                state = "Torque Unfeasible"

            sol += f"State: {state}\n"

            print(sol)

            totElapsed += elapsed
            sumScores  += sNodeAccum
    
            mno.writeTourSol(method, scenario, inst, pi, tour, cfg, pallets, consJK, True, surplus)

    # print(f"Elapsed per instance: {totElapsed/len(instances):.0f}")
    # print(f"Elapsed per instance: {sumScores/len(instances):.0f}")


