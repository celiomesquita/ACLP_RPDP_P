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
mno.NCPU  =   int(sys.argv[3])
limit     = float(sys.argv[4])
mno.DATA  =    f"{sys.argv[5]}"

# scenario = 1

# method = "Shims"
# method = "Shims_p"
# method = "ACO"
# method = "ACO_p"
# method = "Greedy"

# limit = 0.95


cfg = mno.Config(scenario)                                      

dists = mno.loadDistances()

costs = [[0.0 for _ in dists] for _ in dists]

for i, cols in enumerate(dists):
    for j, value in enumerate(cols):
        costs[i][j] = cfg.kmCost*value

pallets = mno.loadPallets(cfg)


if method == "Shims_p":

    factor = [0.32,0.42,0.52,0.62,1.,1.,1.]

    if scenario > 1: # larger aircraft
        factor = [1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,0.9,0.9,0.7,0.7]

    # torque = 0.
    for i, p in enumerate(pallets):
        pallets[i].W *= factor[i]
        # print(f"{p.ID}\t{p.D}\t{p.V}\t{p.W}")
        # torque += p.D * p.W 
    # print(f"\nTorque = {torque/cfg.maxTorque:.2f}\n")

""""""
# pallets capacity
cfg.weiCap = 0
cfg.volCap = 0
for p in pallets:
    cfg.weiCap += p.W
    cfg.volCap += p.V

# smaller aircrafts may have a payload lower than pallets capacity
if cfg.weiCap > cfg.payload:
    cfg.weiCap = cfg.payload   

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

print(f"{numItems} items")

mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

print("Dests: ",end="")
for p in pallets:
    print(f"{mno.CITIES[p.Dests[k]]} ", end='')
print()

E = []

startNodeTime = time.perf_counter()

if method == "Greedy":
    E = greedy.Solve(pallets, items, cfg, k)

if method == "Shims_p":
    E = shims_p.Solve(pallets, items, cfg, k, limit)

if method == "Shims":
    E = shims.Solve(pallets, items, cfg, k, limit)

if method == "ACO":
    E =   aco.Solve( pallets, items, startNodeTime, cfg, k, limit)

if method == "ACO_p":
    E = aco_p.Solve( pallets, items, startNodeTime, cfg, k, limit)

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
""""""
