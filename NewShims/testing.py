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

    vol       = [0]*len(nodes)
    PalConsol = [0]*len(nodes)
    max   = 0
    total = 0

    # all items from unnatended nodes (no consolidated)
    for it in items:
        # the items from this node
        if it.Frm == nodes[k].ID:
            d = it.To
            if d in L_k:
                vol[d] += it.V
                total  += it.V
                if vol[d] > max:
                    max = d
        
    for n in nodes:
        if vol[n.ID] > 0:
            np = math.floor( len(pallets) * vol[n.ID] / total)
            # quant = max(1, np - PalConsol[n.ID])
            quant = np - PalConsol[n.ID]
            count = 0
            for p in pallets:
                if count == quant:
                    break
                if p.Dests[k] == -1:
                    pallets[p.ID].Dests[k] = n.ID
                    count += 1

    for p in pallets:
        if p.Dests[k] == -1:
            pallets[p.ID].Dests[k] = max


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

dists = mno.loadDistances()

costs = [[0.0 for _ in dists] for _ in dists]

for i, cols in enumerate(dists):
    for j, value in enumerate(cols):
        costs[i][j] = cfg.kmCost*value

pallets = shims_mp.loadPallets(cfg)

# pallets capacity
cfg.weiCap = 0
cfg.volCap = 0
for p in pallets:
    cfg.weiCap += p.W
    cfg.volCap += p.V

# smaller aircrafts may have a payload lower than pallets capacity
if cfg.weiCap > cfg.payload:
    cfg.weiCap = cfg.payload   

solElapsed = 0
solScore   = 0

# solution global torque to be shared and changed by all pallets concurrently
solTorque = mp.Value('d') 
solTorque.value = 0.0

for inst in instances:    

    tours = mno.getTours(cfg.numNodes-1, costs, 0.25)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 1 # the first node after the base

    node = tour.nodes[k]
    print(f"ICAO node: {node.ICAO}")
    
    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    # load items parameters from this node and problem instance, that go to unnatended
    items = mno.loadNodeItems(scenario, inst, node, unattended, surplus)
    numItems = len(items)
    print(f"number of items: {numItems}")

    # load consolidated generated in the previous node
    prevNode = tour.nodes[k-1]
    cons = loadNodeCons(surplus, scenario, inst, pi, prevNode, numItems ) # numItems = first cons ID

    if prevNode.ID < len(mno.CITIES):
        print(f"\n-----Loaded from tour {pi} {mno.CITIES[prevNode.ID]} -----")
        print("P\tW\tS\tV\tFROM\tTO")
        for c in cons:
            print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                    c.P, c.W, c.S, c.V, mno.CITIES[c.Frm], mno.CITIES[c.To]))
    print()

    # consolidated contents not destined to this point are kept on board
    kept = []
    for c in cons:
        if c.To in unattended:
            kept.append(c)
        

    print("P\tW\tS\tV\tFROM\tTO")
    for c in kept:
        print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                c.P, c.W, c.S, c.V, mno.CITIES[c.Frm], mno.CITIES[c.To]))
    print("Pallets positions to be defined.\n")

    # optimize consolidated positions to minimize CG deviation
    optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "CBC", k)
    
    print("P\tW\tS\tV\tFROM\tTO")
    for c in kept:
        print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                c.P, c.W, c.S, c.V, mno.CITIES[c.Frm], mno.CITIES[c.To]))
    print("Pallets positions defined.\n")

    # set pallets destinations with items and consolidated to be delivered
    setPalletsDestinations(items, pallets, tour.nodes, k, unattended)
    print("Pallets destinations set.\n") 

    for i, p in enumerate(pallets):
        for it in kept:
            if it.P == p.ID:
                pallets[i].PCW += it.W
                pallets[i].PCV += it.V
                pallets[i].PCS += it.S
                solTorque.value += float(it.W) * float(p.D)

        print(f"{pallets[i].Dests[k]}\t{pallets[i].PCW}\t{pallets[i].PCV:.2f}\t{pallets[i].PCS}")
    print(f"Pallets are now with accumulated values defined. Torque: {solTorque.value:.2f}\n")

    
    E = []

    startNodeTime = time.perf_counter()

    if method == "Shims_mp":
        E = shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque)

    if method == "Shims":            
        E = shims_mp.Solve(pallets, items, cfg, k, limit, secBreak, "s")         

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

        solElapsed += elapsed
        solScore  += sNodeAccum

        mno.writeTourSol(method, scenario, inst, pi, tour, cfg, pallets, consJK, True, surplus)

# print(f"Elapsed per instance: {solElapsed/len(instances):.0f}")
# print(f"Elapsed per instance: {solScore/len(instances):.0f}")


