import methods as mno
import numpy as np
import time
import shims_p
import shims
import aco
import aco_p
import greedy

# mno.DATA = "data20"
# mno.DATA = "data50"
# mno.DATA = "data100"

import sys
scenario  =   int(sys.argv[1])
method    =    f"{sys.argv[2]}"
mno.DATA  =    f"{sys.argv[3]}"

# scenario = 1

# method = "Greedy"
# method = "Shims"
# method = "Shims_p"
# method = "ACO"
# method = "ACO_p"

# data20 Shims best limit = 0.10 | 0.02s-0.04s
# data20 ACO   best limit = 0.50
# data20 ACO-p best limit = 0.40
# data50 Shims best limit = 0.10
# data50 ACO   best limit = 0.90 | 1.82s
# data50 ACO-p best limit = 0.90 | 0.77s

minLim = 0.75 # Shims_p
limit  = 0.95 # Shims and ACO
secBreak = 0.7

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

numTries = 3
if method == "Shims" or method == "Shims_p" or method=="Greedy": # the solution is always the same
    numTries = 1

tries = numTries

totElapsed = 0
sumScores = 0

# numCpus = [4,8,12,16,20,24,28,32,36,40,44,48]
numCpus = [1,2,4,6,8,10]
# numCpus = [8]

if method == "Shims" or method == "ACO" or method == "Greedy":
    numCpus = [1]

times = [0 for _ in numCpus]
scores = [0 for _ in numCpus]

while tries:
    tries -= 1

    for ix, numCPU in enumerate(numCpus):

        tours = mno.getTours(cfg.numNodes-1, costs, 0.25)

        pi = 0 # the first, not necessarily the best

        tour = tours[pi]

        k = 0 # the base

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        node = tour.nodes[k]
        print(node.ICAO)


        # load items parameters from this node and problem instance, that go to unnatended
        items = mno.loadNodeItems(scenario, 1, node, unattended)

        numItems = len(items)

        print(f"{numItems} items and {numCPU} processes")

        mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

        print("Dests: ",end="")
        for p in pallets:
            print(f"{mno.CITIES[p.Dests[k]]} ", end='')
        print()

        E = []

        startNodeTime = time.perf_counter()

        if method == "Greedy":
            E = greedy.Solve(pallets, items, cfg, k)

        if method == "Shims_p": # 6 CPU
            E = shims_p.Solve(pallets, items, cfg, k, minLim, numCPU)

        if method == "Shims":
            E = shims.Solve(pallets, items, cfg, k, limit)

        if method == "ACO":
            E =   aco.Solve( pallets, items, startNodeTime, cfg, k, limit, secBreak)

        if method == "ACO_p":
            E = aco_p.Solve( pallets, items, startNodeTime, cfg, k, limit, numCPU, secBreak)

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
                times[ix]  += elapsed
                scores[ix] += sNodeAccum # * (2-abs(epsilom))


print(f"average elapsed = {totElapsed/(numTries):.2f} | average score = {sumScores/numTries:.0f}")

if method == "Shims_p" or method == "ACO_p":

    text = "np,time,score\n"
    for i, _ in enumerate(numCpus):
        text += f"{numCpus[i]},{times[i]:.2f},{scores[i]:.0f}\n"

    fname = f"./latex/csv/{method}{scenario}{mno.DATA}.csv"

    writer = open(fname,'w+') # + creates the file, if not exists

    try:
        writer.write(text)
    finally:
        writer.close() 

    print(text)
