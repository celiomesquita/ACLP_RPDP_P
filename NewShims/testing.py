import methods as mno
import numpy as np
import time
import multiprocessing as mp
import os
import math

import shims_mp
import optcgcons

# some pallets already have their destinations set by the consolidated
def setPalletsDestinations(items, pallets, nodes, k, L_k):

    itemVols = [0]*len(nodes)
    consNums = [0]*len(nodes)
    lastDest  = 0
    totalVol = 0

    # all items to unnatended nodes (no consolidated)
    for it in items:
        # the items from this node
        if it.Frm == nodes[k].ID and it.P == -1:
            d = it.To
            if d in L_k:
                itemVols[d] += it.V
                totalVol       += it.V
                if itemVols[d] > lastDest:
                    lastDest = d

    # all consolidated to unnatended nodes
    for it in items:
        # the consolidated from this node
        if it.Frm == nodes[k].ID and it.P == -2:    
            d = it.To
            consNums[d] += 1
            if d in L_k:
                itemVols[d] += it.V
                totalVol    += it.V
                if itemVols[d] > lastDest:
                    lastDest = d

    for n in nodes:
        if itemVols[n.ID] > 0:
            np = math.floor( len(pallets) * itemVols[n.ID] / totalVol)
            # quant = lastDest(1, np - consNums[n.ID])
            quant = np - consNums[n.ID]
            count = 0
            for p in pallets:
                if count == quant:
                    break
                if p.Dests[k] == -1: # destination not set
                    pallets[p.ID].Dests[k] = n.ID
                    count += 1

    for p in pallets:
        if p.Dests[k] == -1:  # destination not set
            pallets[p.ID].Dests[k] = lastDest

# item or consolidated
class Item(object):
    """
    A candidate item "j" to be loaded on a pallet "i" if X_ij == 1
    """
    def __init__(self, id, p, w, s, v, frm, to):
        self.ID = id
        self.P  = p  # -1 if an item, -2 if a consollidated, or pallet ID.
        self.W  = w  # weight
        self.S  = s  # score
        self.V  = v  # volume
        self.Frm = frm  # from
        self.To = to # destination

def loadNodeCons(DATA, scenario, instance, pi, node, id):
    """
    Loads consolidated contents file for this instance, tour and node k
    """
    dirname = f"./{DATA}/scenario_{scenario}/instance_{instance}"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass    

    fname = f"{dirname}/cons_{pi}_{node.ID}.txt"

    reader = open(fname,"r")
    lines = reader.readlines() 

    cons = []
    try:
        for line in lines:
            cols = line.split()
            w   =   int(cols[0])
            s   =   int(cols[1])
            v   = float(cols[2])
            frm =   int(cols[3])
            to  =   int(cols[4])         
            if w > 0: #           P = -2 consolidated
                cons.append( Item(id, -2, w, s, v, frm, to) )
                id += 1
    finally:
        reader.close()

    return cons

surplus = "data20"
# surplus = "data50"
# surplus = "data100"

method = "Shims_mp"
# method = "Shims"

scenario = 1

# instances = [1,2,3,4,5,6,7]
instances = [1]

# limit    = 0.95 # best
limit    = 0.95
secBreak = 0.7 # seconds

cfg = mno.Config(scenario)                                      

# --- distances and costs matrix ---
dists = mno.loadDistances()

costs = [[0.0 for _ in dists] for _ in dists]

for i, cols in enumerate(dists):
    for j, value in enumerate(cols):
        costs[i][j] = cfg.kmCost*value

pallets = shims_mp.loadPallets(cfg)

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

    tours = mno.getTours(cfg.numNodes-1, costs, 0.25)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 1 # the first node after the base

    node = tour.nodes[k]
    # print(f"ICAO node: {node.ICAO}")
    
    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    # load items parameters from this node and problem instance, that go to unnatended
    items = mno.loadNodeItems(scenario, inst, node, unattended, surplus)
    numItems = len(items)
    # print(f"number of items: {numItems}")

    # load consolidated generated in the previous node
    prevNode = tour.nodes[k-1]
    cons = loadNodeCons(surplus, scenario, inst, pi, prevNode, numItems ) # numItems = first cons ID

    # if prevNode.ID < len(mno.CITIES):
    #     print(f"\n-----Loaded in {mno.CITIES[prevNode.ID]} -----")
    #     print("ID\tP\tW\tS\tV\tFROM\tTO")
    #     for c in cons:
    #         print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{mno.CITIES[c.Frm]}\t{mno.CITIES[c.To]}")
    # print(f"({numItems} items to embark)")


    # consolidated contents not destined to this point are kept on board ...
    kept = []
    for c in cons:
        if c.To in unattended:
            c.ID = numItems
            kept.append(c) #... and included in the items set
            numItems += 1

    # print(f"\n----- Kept on board at {mno.CITIES[node.ID]} -----")        
    # print("ID\tP\tW\tS\tV\tFROM\tTO")
    # for c in kept:
    #     print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{mno.CITIES[c.Frm]}\t{mno.CITIES[c.To]}")
    # print(f"Kept positions to be defined: ({numItems} items to embark)\n")

    # optimize consolidated positions to minimize CG deviation
    optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)
    # pallets destinations are also set, according to kept on board in new positions

    # Kept P is not -2 anymore, but the pallet ID.
    
    # print("ID\tP\tW\tS\tV\tFROM\tTO")
    for c in kept:
        # consolidated are appended to the items set
        items.append(c)
        # print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{mno.CITIES[c.Frm]}\t{mno.CITIES[c.To]}")
    # print(f"Kept positions defined ({numItems} items to embark)\n")

    # print("ID\tDest\tPCW\tPCV\tPCS")
    # for p in pallets:
    #     print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
    # print("Pallets destinations to be defined.\n")

    # set pallets destinations with items and consolidated to be delivered
    setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

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

    startNodeTime = time.perf_counter()

    if method == "Shims_mp":
        E = shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque, solItems)

    if method == "Shims":            
        E = shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "s", solTorque, solItems)         

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

            for i in np.arange(numItems):

                if E[j][i] == 1:
                    consJK[j][k].W += items[i].W
                    consJK[j][k].V += items[i].V
                    consJK[j][k].S += items[i].S

            consNodeT[j] = consJK[j][k]

        sNodeAccum = 0.
        wNodeAccum = 0.
        vNodeAccum = 0.
        sol = ""

        for i, p in enumerate(pallets):
            sNodeAccum += float(consJK[i][k].S)
            wNodeAccum += float(consJK[i][k].W)
            vNodeAccum += float(consJK[i][k].V)

        epsilom = solTorque.value/cfg.maxTorque

        sol += f"Score: {sNodeAccum}\t"
        sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
        sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
        sol += f"Torque: {epsilom:.2f}\n"
        sol += f"Elapsed: {elapsed:.2f}\n"

        state = "Feasible"

        if wNodeAccum/cfg.weiCap > 1.0:
            state = "Weight Unfeasible"

        if vNodeAccum/cfg.volCap > 1.0:
            state = "Volume Unfeasible"

        if abs(epsilom) > 1.0:
            state = "Torque Unfeasible"

        sol += f"State: {state}\n"

        print(sol)

        solElapsed += elapsed
        solScore   += sNodeAccum

        mno.writeTourSol(method, scenario, inst, pi, tour, cfg, pallets, consJK, True, surplus)

# print(f"Elapsed per instance: {solElapsed/len(instances):.0f}")
# print(f"Elapsed per instance: {solScore/len(instances):.0f}")
