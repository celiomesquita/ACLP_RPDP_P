import numpy as np
import multiprocessing as mp
import time
import math

import knapsack as kp
import common
import optcgcons

# A Shims fits in a pallet slack
class Shims(object):
    def __init__(self, pallet, whipLen):
        self.Pallet = pallet # this shims is one of the possibles for this pallet
        self.SCW    = 0      # shims current weight (must fit the slack)
        self.SCV    = 0.0    # shims current volume (must fit the slack)
        self.SCS    = 0      # shims current score
        self.SCT    = 0.     # shims current torque
        self.Items  = [None for _ in np.arange(whipLen)]

    def putItem(self, item, w):
                 
            self.SCW += item.W
            self.SCV += item.V
            self.SCS += item.S
            self.SCT += float(item.W) * float(self.Pallet.D)
            self.Items[w] = item

    def isFeasible(self, item, k, nodeTorque, cfg, lock): # check constraints

        if item.To != self.Pallet.Dest[k]:
            return False

        if self.Pallet.PCV + self.SCV + item.V > self.Pallet.V * 1.02:
            return False

        if self.Pallet.PCW + self.SCW + item.W > self.Pallet.W:
            return False

        deltaTau = float(item.W) * float(self.Pallet.D)
        ret = True
        with lock:
            newTorque = abs(nodeTorque.value + self.SCT + deltaTau)
            if newTorque > abs(nodeTorque.value + self.SCT) and newTorque > cfg.maxTorque:
                ret = False
        return ret

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, k, nodeTorque, solDict, cfg, surplus, itemsDict, lock, tipo):

    maxVol = pallet.V * surplus

    vol = 0.
    whip = []
    N = len(items)
    i = pallet.ID
   
    with lock:   
        for item in items:
            j = item.ID
            if itemsDict["mpItems"][j] == 0:
                vol += item.V
                whip.append(item)
                if vol > maxVol:
                    break

    # create the first shim
    sh = Shims(pallet, len(whip))
    Set = [sh]

    if tipo == "KP":
        capacity = math.floor((pallet.V - pallet.PCV)*100)
        if capacity > 0:
            volumes = [0]*len(whip)
            scores  = [0]*len(whip)
            for w, item in enumerate(whip):
                volumes[w] = math.floor(item.V*100)
                scores[w]  = item.S
                    
            indexes = kp.Solve(capacity, volumes, scores)

            for i in indexes:
                item = whip[i]
                if pallet.isFeasible(item, 1.1, k, nodeTorque,  cfg, itemsDict, lock):
                    pallet.putItem(item, nodeTorque, solDict, N, itemsDict, lock)

    if tipo == "FFD":
        # First Fit Decrease - faster than KP
        whip.sort(key=lambda x: abs(x.V), reverse=True)

        for w, item in enumerate(whip):
            newShims = True
            for sh in Set:
                if sh.isFeasible(item, k, nodeTorque, cfg, lock):
                    sh.putItem(item, w)
                    newShims = False
                    break

            if newShims:
                sh = Shims(pallet, len(whip)) # create a new Shim
                if sh.isFeasible(item, k, nodeTorque, cfg, lock):
                    sh.putItem(item, w)
                Set.append(sh)
        # select the best Shim
        bestScore = 0
        bestIndex = 0
        for i, shims in enumerate(Set):
            if shims.SCS > bestScore:
                bestScore = shims.SCS
                bestIndex = i
        # put the best Shim in the solution
        for item in Set[bestIndex].Items:
            if item != None and pallet.isFeasible(item, 1.02, k, nodeTorque,  cfg, itemsDict, lock):
                pallet.putItem(item, nodeTorque, solDict, N, itemsDict, lock)


def Solve(pallets, items, cfg, k, threshold, secBreak, mode, nodeTorque, solDict, itemsDict, tipo):

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    surplus = 1. + 3. *(1. - threshold)

    lock  = mp.Lock()
    counter = 0

    if mode == "Parallel":
    
        procs = [None for _ in pallets] # each pallets has its own process

        # parallel greedy phase
        for i, _ in enumerate(pallets):
            procs[i] = mp.Process( target=common.fillPallet, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, threshold, itemsDict, lock, 1.0) ) # torque surplus
            time.sleep(0.001)
            procs[i].start()
        
        for proc in procs:
            proc.join()

        optcgcons.minCGdev(pallets, k, nodeTorque, cfg) # made no significant difference

        # parallel shims phase
        for i, p in enumerate(pallets):
            procs[i] = mp.Process( target=getBestShims, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, surplus, itemsDict, lock, tipo) )
            time.sleep(0.001)                 
            procs[i].start()
                
        for p in procs:
            p.join()

    else: # serial
        # sort ascendent by CG distance
        # pallets.sort(key=lambda x: abs(x.D)) # deactivated because of torque surplus
        for i, _ in enumerate(pallets):
            # fill until the threshold                                                               torque surplus
            common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, threshold, itemsDict, lock, 2.)

            optcgcons.minCGdev(pallets, k, nodeTorque, cfg)
           
            # get the best Shims for the pallet
            getBestShims( pallets[i], items, k, nodeTorque, solDict, cfg, surplus,   itemsDict, lock, tipo)

    # try to complete the pallet
    for i, _ in enumerate(pallets):
        counter += common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, 1.02, itemsDict, lock)

    print(f"{mode}: {counter} items inserted in post local search.")


if __name__ == "__main__":

    import sys
    seed         = sys.argv[1]
    instance     = sys.argv[2] # /home/celio/Projects/ACLP_RPDP_P/data20/scenario_2/instance_3/items.txt'
    volThreshold = float(sys.argv[3])
    tipo         = sys.argv[4]

    values = instance.split('/')

    lenv = len(values)

    # /home/celio/Projects/ACLP_RPDP_P/data20/scenario_2/instance_3/items.txt'
    #                                  data20/scenario_2/instance_3/items.txt'

    if lenv == 4:
        surplus   = values[0]
        scenario = int(values[1][-1:])
        instance = int(values[2][-1:]) 

    if lenv == 9:
        surplus   = values[5]
        scenario = int(values[6][-1:])
        instance = int(values[7][-1:])         

    # method = "mpShims" # best volThreshold =  data50(0.9781)
    method = "Shims"  # best volThreshold: data20 (0.98)

    secBreak = 1.8

    # tipo = "FFD"

    # --- distances and costs matrix ---
    dists = common.loadDistances("params/distances.txt")

    costs = [[0.0 for _ in dists] for _ in dists]

    cfg = common.Config(scenario)                                      

    for i, cols in enumerate(dists):
        for j, dist in enumerate(cols):
            costs[i][j] = cfg.kmCost*dist

    pallets, rampDistCG = common.loadPallets(cfg)
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

    tours = common.getTours(cfg.numNodes-1, costs, 1.0)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    # a matrix for all consolidated in the tour
    consol = [
                [ common.Item(-1, -2, 0, 0, 0., -1, -1)
                for _ in tour.nodes ]
                for _ in pallets # a consolidated for each pallet
            ]

    k = 0 # the base

    for i, p in enumerate(pallets):
        pallets[i].reset(cfg.numNodes)

    node = tour.nodes[k]

    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    # load items parameters from this node and problem instance, that go to unnatended
    items = common.loadNodeItems(scenario, instance, node, unattended, surplus)
    N = len(items)


    # solution global torque to be shared and changed by all pallets concurrently
    nodeTorque = mp.Value('d', 0.0) # a multiprocessing double type variable

    # initialize- the accumulated values
    sNodeAccum = 0.
    wNodeAccum = 0.
    vNodeAccum = 0.

    # set empty pallets (-1) destinations based on the items to embark
    common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

    # to control solution items
    M = len(pallets)
    solMatrix = mp.Array('i', [0 for _ in np.arange(N*M)] )
    mpItems   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

    solDict   = dict(solMatrix=solMatrix)
    itemsDict = dict(mpItems=mpItems)

    objValue = 0.0

    if method == "mpShims":
        Solve(pallets, items, cfg, k, volThreshold, secBreak, "p", nodeTorque, solDict, itemsDict, tipo)

    if method == "Shims":            
        Solve(pallets, items, cfg, k, volThreshold, secBreak, "s", nodeTorque, solDict, itemsDict, tipo)         

    # Validate the solution for this node

    Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)

    for i, row in enumerate(Y):
        for j, X_ij in enumerate(row):
            if X_ij:
                consol[i][k].ID  = j+N
                consol[i][k].Frm = node.ID
                consol[i][k].To  = pallets[i].Dest[k]                   
                consol[i][k].W += items[j].W
                consol[i][k].V += items[j].V
                consol[i][k].S += items[j].S

                sNodeAccum += float(items[j].S)

    # print(-1 * sNodeAccum) # to be captured by iRace. -1 because its maximization


