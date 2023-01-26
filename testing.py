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
def testingGetCons(N, rndm):
    """
    Testing consolidated from cons_0_0.txt
    """
    cons = []

    lines = [
        [ 620,  406,  4.121, 0, 2],
        [2595, 2436, 13.690, 0, 2],
        [3282, 3081, 13.689, 0, 2],
        [4217, 3856, 13.674, 0, 2],
        [2558, 3180, 13.692, 0, 1],
        [3526, 3630, 13.691, 0, 1],
        [4348, 3509, 13.697, 0, 1]
    ]
    # lines = [
    #     [1230, 80,  9.0, 0, 1], # from the base to the first node
    #     [3560, 90, 10.0, 0, 2], # from the base to the second node
    #     [2340, 70, 11.0, 0, 1],
    #     [2360, 60, 12.0, 0, 2],
    #     [1250, 50, 13.0, 0, 1],
    #     [2540, 40,  8.0, 0, 2],
    #     [3540, 30,  7.0, 0, 1]
    # ]

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

method    = "Shims"
# method    = "mpShims"
# method    = "ACO"
# method    = "mpACO"
# method    = "GRB"

scenario = 1

# instances = [1,2,3,4,5,6,7]
instances = [1]

# limit    = 0.95 # best
limit    = 0.25
secBreak = 0.7 # seconds
# secBreak = 60


# --- distances and costs matrix ---
dists = common.loadDistances()

costs = [[0.0 for _ in dists] for _ in dists]

cfg = common.Config(scenario)                                      

for i, cols in enumerate(dists):
    for j, dist in enumerate(cols):
        costs[i][j] = cfg.kmCost*dist

pallets = common.loadPallets(cfg)
lock = mp.Lock

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

    tours = common.getTours(cfg.numNodes-1, costs, 1.0)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    # a matrix for all consolidated in the tour
    consol = [
                [ common.Item(-1, -2, 0, 0, 0., -1, -1)
                for _ in tour.nodes ]
                for _ in pallets # a consolidated for each pallet
            ]

    k = 1 # the first node after the base: cons_0_0.txt

    for i, p in enumerate(pallets):
        pallets[i].reset(cfg.numNodes)

    node = tour.nodes[k]
    print(f"ICAO current node: {node.ICAO}")
    
    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    # load items parameters from this node and problem instance, that go to unnatended
    items = common.loadNodeItems(scenario, inst, node, unattended, surplus)
    N = len(items)
    print(f"({N} items to embark)")

    # load consolidated generated in the previous node
    prevNode = tour.nodes[k-1]

    # N = first cons ID
    # cons = common.loadNodeCons(surplus, scenario, inst, pi, prevNode, N )
    # cons = testingGetCons(N, True) # for testing only
    cons = testingGetCons(N, False) # False: no randomness in consolidated generation

    # --- from main
    # cons = []
    # for i, _ in enumerate(pallets):
    #         cons.append( consol[i][k-1] )    

    if prevNode.ID < len(common.CITIES):
        print(f"\n-----Loaded in {common.CITIES[prevNode.ID]} -----")
        print("ID\tP\tW\tS\tV\tFROM\tTO")
        for c in cons:
            print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")

    # consolidated contents not destined to this point are kept on board ...
    kept = []
    cid = N
    for c in cons:
        if c.To in unattended:
            c.ID = cid
            kept.append(c) #... and included in the items set
            cid += 1
    print(f"({len(kept)} consolidated kept on board)")

    print(f"\n----- Kept on board at {common.CITIES[node.ID]} -----")        
    print("ID\tP\tW\tS\tV\tFROM\tTO")
    for c in kept:
        print(f"{c.ID}\t{c.P}\t{c.W}\t{c.S}\t{c.V:.1f}\t{common.CITIES[c.Frm]}\t{common.CITIES[c.To]}")
    print(f"Kept positions to be defined\n")

    # solution global torque to be shared and changed by all pallets concurrently
    solTorque = mp.Value('d', 0.0) # a multiprocessing double type variable

    # initialize- the accumulated values
    sNodeAccum = 0.
    wNodeAccum = 0.
    vNodeAccum = 0.

    # Optimize consolidated positions to minimize CG deviation.
    # Pallets destinations are also set, according to kept on board in new positions
    # Kept P is not -2 anymore, but the pallet ID.
    if len(kept) > 0:
        optcgcons.OptCGCons(kept, pallets, cfg.maxTorque, "GRB", k)

        # N: number of items to embark
        # put the consolidated on their assgined pallets (OptCGCons)
        for c in kept:
            for i, p in enumerate(pallets):
                if c.P == p.ID:
                    pallets[i].putConsol( c, solTorque)

                    # update the consolidated of the current node "k"
                    consol[i][k].ID  = j+N
                    consol[i][k].Frm = node.ID
                    consol[i][k].To  = pallets[i].Dests[k]
                    consol[i][k].W  += c.W
                    consol[i][k].V  += c.V
                    consol[i][k].S  += c.S

                    # update the accumulated values
                    sNodeAccum += c.S
                    wNodeAccum += c.W
                    vNodeAccum += c.V

    print("ID\tDest\tPCW\tPCV\tPCS")
    for p in pallets:
        print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
    print("Pallets destinations to be defined.\n")

    # set empty pallets (-1) destinations based on the items to embark
    common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

    print("ID\tDest\tPCW\tPCV\tPCS")
    for p in pallets:
        print(f"{p.ID}\t{p.Dests[k]}\t{p.PCW}\t{p.PCV:.2f}\t{p.PCS}")
    print("Pallets destinations defined.\n")
   
    startNodeTime = time.perf_counter()

    # to control solution items
    M = len(pallets)
    solMatrix = mp.Array('i', [0 for _ in np.arange(N*M)] )
    mpItems   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

    solDict   = dict(solMatrix=solMatrix)
    itemsDict = dict(mpItems=mpItems)
 
    if method == "mpShims":
        mpShims.Solve(pallets, items, cfg, k, limit, secBreak, "p", solTorque, solDict, itemsDict)

    if method == "Shims":            
        mpShims.Solve(pallets, items, cfg, k, limit, secBreak, "s", solTorque, solDict, itemsDict)         

    if method == "mpACO":       
        mpACO.Solve(pallets,   items, cfg, k, limit, secBreak, "p", solTorque, solDict, itemsDict) 

    if method == "ACO":       
        mpACO.Solve(pallets,   items, cfg, k, limit, secBreak, "s", solTorque, solDict, itemsDict) 

    if method == "GRB":       
        mipGRB.Solve(pallets,  items, cfg, k,        secBreak,      solTorque, solDict, itemsDict) 
    
    elapsed = time.perf_counter() - startNodeTime

    # Validate the solution for this node

    Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)

    for i, row in enumerate(Y):
        for j, X_ij in enumerate(row):
            if X_ij:
                consol[i][k].ID  = j+N
                consol[i][k].Frm = node.ID
                consol[i][k].To  = pallets[i].Dests[k]                   
                consol[i][k].W += items[j].W
                consol[i][k].V += items[j].V
                consol[i][k].S += items[j].S

                sNodeAccum += float(items[j].S)
                wNodeAccum += float(items[j].W)
                vNodeAccum += float(items[j].V)

    feasible = "Feasible"                
    for n in itemsDict["mpItems"]:
        if n > 1:
            feasible = "Unfeasible!!!"
            break    

    epsilom = solTorque.value/cfg.maxTorque

    vol = vNodeAccum/cfg.volCap
    wei = wNodeAccum/cfg.weiCap

    sol  = f"Score: {sNodeAccum:.0f}\t"
    sol += f"Weight: {wei:.2f}\t"
    sol += f"Volume: {vol:.2f}\t"
    sol += f"Torque: {epsilom:.2f}\t"
    sol += f"Items: {feasible}\n"
    sol += f"Elapsed: {elapsed:.2f}\n"

    print(f"Testing solution ----- \n{sol}\n")

    # solElapsed += elapsed
    # solScore   += sNodeAccum

    consNodeT = [None for _ in pallets]        
    for i, p in enumerate(pallets):
        consNodeT[i] = consol[i][k]

    # write consolidated contents from this node in file
    common.writeNodeCons(scenario, inst, consNodeT, pi, node, surplus, epsilom, wei, vol)

    # common.writeTourSol(method, scenario, inst, pi, tour, cfg, pallets, consol, True, surplus)

# print(f"Elapsed per instance: {solElapsed/len(instances):.0f}")
# print(f"Elapsed per instance: {solScore/len(instances):.0f}")
