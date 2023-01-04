import common
import numpy as np
import time
import multiprocessing as mp

import shims_mp
import aco_mp
import optcgcons

surplus = "data20"
# surplus = "data50"
# surplus = "data100"

method = "ACO_mp"
# method = "ACO"
# method = "Shims_mp"
# method = "Shims"

scenario = 1

# instances = [1,2,3,4,5,6,7]
instances = [1]

# limit    = 0.95 # best
limit    = 0.95
secBreak = 0.7 # seconds

cfg = common.Config(scenario)                                      

# --- distances and costs matrix ---
dists = common.loadDistances()

costs = [[0.0 for _ in dists] for _ in dists]

for i, cols in enumerate(dists):
    for j, value in enumerate(cols):
        costs[i][j] = cfg.kmCost*value

pallets = common.loadPallets(cfg)

# pallets capacities
cfg.weiCap = 0
cfg.volCap = 0
for p in pallets:
    cfg.weiCap += p.W
    cfg.volCap += p.V

# smaller aircrafts may have a payload lower than pallets capacity
if cfg.weiCap > cfg.payload:
    cfg.weiCap = cfg.payload   

# solution global parameters
solElapsed = 0
solScore   = 0

# solution global torque to be shared and changed by all pallets concurrently
solTorque = mp.Value('d') # a multiprocessing double type variable
solTorque.value = 0.0

for inst in instances:    

    tours = common.getTours(cfg.numNodes-1, costs, 0.25)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 1 # the first node after the base

    node = tour.nodes[k]
    # print(f"ICAO node: {node.ICAO}")
    
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
    
    startNodeTime = time.perf_counter()
 
    if method == "Shims_mp":
        shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque, solItems)

    if method == "Shims":            
        shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "s", solTorque, solItems)         

    if method == "ACO_mp":       
        aco_mp.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque, solItems) 

    if method == "ACO":       
        aco_mp.Solve(pallets, items, cfg, k, limit, secBreak, "s", solTorque, solItems) 

    elapsed = time.perf_counter() - startNodeTime

    # a matrix for all consolidated in the tour
    consol = [
                [ common.Item(-1, -2, 0, 0, 0., -1, -1)
                for _ in tour.nodes ]
                for _ in pallets # a consolidated for each pallet
            ] 

    # Validate the solution for this node

    sNodeAccum = 0.
    wNodeAccum = 0.
    vNodeAccum = 0.
    sol = ""


    palletsCount = [0 for _ in solItems]

    pallets.sort(key=lambda x: x.ID) 

    for j, i in enumerate(solItems):
        if i > -1: # i: pallet index
            consol[i][k].ID  = j+numItems
            consol[i][k].Frm = node.ID
            consol[i][k].To  = p.Dests[k]
            palletsCount[j] += 1

            if palletsCount[j] < 2:

                consol[i][k].W += items[j].W
                consol[i][k].V += items[j].V
                consol[i][k].S += items[j].S

                sNodeAccum += float(items[j].S)
                wNodeAccum += float(items[j].W)
                vNodeAccum += float(items[j].V)

    consNodeT = [None for _ in pallets]        
    for i, p in enumerate(pallets):
        consNodeT[i] = consol[i][k]

    epsilom = solTorque.value/cfg.maxTorque

    sol += f"Score: {sNodeAccum}\t"
    sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
    sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
    sol += f"Torque: {epsilom:.2f}\n"
    sol += f"Elapsed: {elapsed:.2f}\n"

    state = "Feasible"

    if wNodeAccum/cfg.weiCap > 1.01:
        state = "Weight Unfeasible"

    if vNodeAccum/cfg.volCap > 1.01:
        state = "Volume Unfeasible"

    if abs(epsilom) > 1.01:
        state = "Torque Unfeasible"

    sol += f"State: {state}\n"

    print(sol)

    solElapsed += elapsed
    solScore   += sNodeAccum

    common.writeTourSol(method, scenario, inst, pi, tour, cfg, pallets, consol, True, surplus)

# print(f"Elapsed per instance: {solElapsed/len(instances):.0f}")
# print(f"Elapsed per instance: {solScore/len(instances):.0f}")
