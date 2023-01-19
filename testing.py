import common
import numpy as np
import time
import multiprocessing as mp
import random
import mpShims
import mpACO
import optcgcons
import mipGRB

# for testing only
def getCons(N, rndm):
    """
    Testing consolidated from cons_0_0.txt
    """
    cons = []

    lines = [
        [1230, 80,  9.0, 0, 1], # from the base to the first node
        [3560, 90, 10.0, 0, 2], # from the base to the second node
        [2340, 70, 11.0, 0, 1],
        [2360, 60, 12.0, 0, 2],
        [1250, 50, 13.0, 0, 1],
        [2540, 40,  8.0, 0, 2],
        [3540, 30,  7.0, 0, 1]
    ]

    id = N
    for line in lines:
        w   =   int(line[0])
        s   =   int(line[1])
        v   = float(line[2])
        frm =   int(line[3])
        to  =   int(line[4])         
        cons.append( common.Item(id, -2, w, s, v, frm, to) )
        id += 1

    if rndm:
        for i, _ in enumerate(cons):
            cons[i].W = random.uniform(2000, 4000)
            cons[i].V = random.uniform(7, 12) 
            cons[i].S = random.uniform(30, 80) 

    return cons


# surplus = "data20"
surplus = "data50"
# surplus = "data100"
#
# method = "mpACO"
method = "ACO"
# method = "mpShims"
# method = "Shims"
# method = "GRB"

scenario = 1

# instances = [1,2,3,4,5,6,7]
instances = [1]

# limit    = 0.95 # best
limit    = 0.95
secBreak = 0.7 # seconds
# secBreak = 60

cfg = common.Config(scenario)                                      

# --- distances and costs matrix ---
dists = common.loadDistances()

costs = [[0.0 for _ in dists] for _ in dists]

for i, cols in enumerate(dists):
    for j, dist in enumerate(cols):
        costs[i][j] = cfg.kmCost*dist

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

for inst in instances:    

    tours = common.getTours(cfg.numNodes-1, costs, 0.25)
    # tours = common.getTours(cfg.numNodes-1, costs, 1.0)

    print(f"{len(tours)} tours")

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 1 # the first node after the base: cons_0_0.txt

    node = tour.nodes[k]
    print(f"ICAO node: {node.ICAO}")
    
    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    # load items parameters from this node and problem instance, that go to unnatended
    items = common.loadNodeItems(scenario, inst, node, unattended, surplus)
    N = len(items)

    # load consolidated generated in the previous node
    prevNode = tour.nodes[k-1]

     # N = first cons ID
    # cons = common.loadNodeCons(surplus, scenario, inst, pi, prevNode, N )
    # cons = getCons(N, True) # for testing only
    cons = getCons(N, False) # False: no randomness in consolidated generation
    # cons = getCons(N, True) 

    if prevNode.ID < len(common.CITIES):
        print(f"\n-----Loaded in {common.CITIES[prevNode.ID]} -----")
        print("ID\tP\tW\tS\tV\tFROM\tTO")
        for c in cons:
            print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
    print(f"({N} items to embark)")

    # consolidated contents not destined to this point are kept on board ...
    kept = []
    for c in cons:
        if c.To in unattended:
            c.ID = N
            kept.append(c) #... and included in the items set
            N += 1

    print(f"\n----- Kept on board at {common.CITIES[node.ID]} -----")        
    print("ID\tP\tW\tS\tV\tFROM\tTO")
    for c in kept:
        print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
    print(f"Kept positions to be defined: ({N} items to embark)\n")

    # optimize consolidated positions to minimize CG deviation
    # if method != "GRB":
    optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)
    # pallets destinations are also set, according to kept on board in new positions

    # Kept P is not -2 anymore, but the pallet ID.
    
    print("ID\tP\tW\tS\tV\tFROM\tTO")
    for c in kept:
        # consolidated are appended to the items set
        items.append(c)
        print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
    print(f"Kept positions defined ({N} items to embark)\n")

    print("ID\tDest\tPCW\tPCV\tPCS")
    for p in pallets:
        print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
    print("Pallets destinations to be defined.\n")

    # set pallets destinations with items and consolidated to be delivered
    common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

    print("ID\tDest\tPCW\tPCV\tPCS")
    for p in pallets:
        print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
    print("Pallets destinations defined.\n")


    # to control solution items
    M = len(pallets)
    solMatrix = mp.Array('i', [0 for _ in np.arange(N*M)] )
    mpItems   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

    # put the kept on board in solution
    for c in kept:
        i = c.P
        j = c.ID
        # solItems[j] = i # consol pallet index
        solMatrix[N*i+j] = 1
        mpItems[j] = 1

    # solution global torque to be shared and changed by all pallets concurrently
    solTorque = mp.Value('d', 0.0) # a multiprocessing double type variable

    # update pallets current parameters and solution torque
    for i, p in enumerate(pallets):
        for c in kept:
            if c.P == p.ID:
                pallets[i].PCW += c.W
                pallets[i].PCV += c.V
                pallets[i].PCS += c.S
                solTorque.value += float(c.W) * float(p.D)

        print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
    print(f"Pallets are now with current values defined. Torque: {solTorque.value/cfg.maxTorque:.2f}\n")
    
    startNodeTime = time.perf_counter()

    solDict     = dict(solMatrix=solMatrix)
    mpItemsDict = dict(mpItems=mpItems)
 
    if method == "mpShims":
        mpShims.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque, solDict, mpItemsDict)

    if method == "Shims":            
        mpShims.Solve(pallets, items, cfg, k, limit, secBreak, "s", solTorque, solDict, mpItemsDict)         

    if method == "mpACO":       
        mpACO.Solve(pallets,   items, cfg, k, limit, secBreak, "p", solTorque, solDict, mpItemsDict) 

    if method == "ACO":       
        mpACO.Solve(pallets,   items, cfg, k, limit, secBreak, "s", solTorque, solDict, mpItemsDict) 

    if method == "GRB":       
        mipGRB.Solve(pallets,  items, cfg, k,        secBreak,      solTorque, solDict, mpItemsDict) 
    
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

    Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)

    for i, row in enumerate(Y):
        for j, X_ij in enumerate(row):
            if X_ij:    
                p = pallets[i]
                consol[i][k].ID  = j+N
                consol[i][k].Frm = node.ID
                consol[i][k].To  = p.Dests[k]

                consol[i][k].W += items[j].W
                consol[i][k].V += items[j].V
                consol[i][k].S += items[j].S

                sNodeAccum += float(items[j].S)
                wNodeAccum += float(items[j].W)
                vNodeAccum += float(items[j].V)

    feasible = "Feasible"                
    for n in mpItemsDict["mpItems"]:
        if n > 1:
            feasible = "Unfeasible!!!"
            break    

    consNodeT = [None for _ in pallets]        
    for i, p in enumerate(pallets):
        consNodeT[i] = consol[i][k]

    epsilom = solTorque.value/cfg.maxTorque

    vol = vNodeAccum/cfg.volCap
    wei = wNodeAccum/cfg.weiCap



    sol += f"Score: {sNodeAccum:.0f}\t"
    sol += f"Weight: {wei:.2f}\t"
    sol += f"Volume: {vol:.2f}\t"
    sol += f"Torque: {epsilom:.2f}\t"
    sol += f"Items: {feasible}\n"
    sol += f"Elapsed: {elapsed:.2f}\n"

    print(f"Testing solution ----- \n{sol}\n")

    # solElapsed += elapsed
    # solScore   += sNodeAccum

    # write consolidated contents from this node in file
    common.writeNodeCons(scenario, inst, consNodeT, pi, node, surplus)

    # common.writeTourSol(method, scenario, inst, pi, tour, cfg, pallets, consol, True, surplus)

# print(f"Elapsed per instance: {solElapsed/len(instances):.0f}")
# print(f"Elapsed per instance: {solScore/len(instances):.0f}")
