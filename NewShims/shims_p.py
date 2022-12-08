# import methods as mno
import numpy as np
import multiprocessing as mp
import time
import math
import knapsack as kp

# A Shims fits in a pallet slack
class Shims(object):
    def __init__(self, pallet, whipLen):
        self.Pallet = pallet # this shims is one of the possibles for this pallet
        self.SCW    = 0      # shims current weight (must fit the slack)
        self.SCV    = 0.0    # shims current volume (must fit the slack)
        self.SCS    = 0      # shims current score
        self.SCT    = 0.     # shims current torque
        self.Items  = [None for _ in np.arange(whipLen)]

    def putItem(self, item, whipPos):
                 
            self.SCW += item.W
            self.SCV += item.V
            self.SCS += item.S
            self.SCT += float(item.W) * float(self.Pallet.D)
            self.Items[whipPos] = item

    def isFeasible(self, item, k, solTorque, cfg): # check constraints

        if item.To != self.Pallet.Dests[k]:
            return False

        if self.Pallet.PCV + self.SCV + item.V > self.Pallet.V:
            return False

        deltaTau = float(item.W) * float(self.Pallet.D)
        if abs(solTorque.value + self.SCT + deltaTau) > cfg.maxTorque:
            return False

        return True        

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, limit, k, solTorque, solItems, cfg, surplus):

    # surplus = 2. - limit
    # surplus = math.exp(-limit) + 0.9
    # print(f"surplus: {surplus:.2f}")

    maxVol = pallet.V * surplus

    vol = 0.
    whip = []
    
    for item in items:
        if solItems[item.ID] == -1: # not allocated in any pallet
            vol += item.V
            whip.append(item)
            if vol > maxVol:
                break

    # create the first shim
    sh = Shims(pallet, len(whip))
    Set = [sh]

    # First Fit Decrease - equivalente ao KP, mas mais rápido
    # included1 = 0
    # included2 = 0
    for whipPos, item in enumerate(whip):
        newShims = True
        for sh in Set:
            if sh.isFeasible(item, k, solTorque, cfg):
                sh.putItem(item, whipPos)
                newShims = False
                # included1 += 1
                break

        if newShims:
            sh = Shims(pallet, len(whip)) # create a new Shim
            if sh.isFeasible(item, k, solTorque, cfg):
                sh.putItem(item, whipPos)
                # included2 += 1
            Set.append(sh)
                
    bestScore = 0
    bestIndex = 0
    for i, shims in enumerate(Set):
        if shims.SCS > bestScore:
            bestScore = shims.SCS
            bestIndex = i

    # print(f"{len(whip)}\t{len(Set)}\t{bestIndex}\t{included1}\t{included2}\t{surplus:.1f}")

    return Set[bestIndex] 


class Pallet(object):
    def __init__(self, id, d, v, w, numNodes):
        self.ID = id
        self.D  = d  # centroid distance to CG
        self.V  = v  # volume limit
        self.W  = w  # weight limit
        self.Dests = np.full(numNodes, -1)
        self.PCW = 0 # pallet current weight
        self.PCV = 0.
        self.PCS = 0.

    def putItem(self, item, solTorque, solItems): # put an item in this pallet

        self.PCW += item.W
        self.PCV += item.V
        self.PCS += item.S

        solTorque.value += float(item.W) * float(self.D)
        solItems[item.ID] = self.ID # mark item as alocated to this pallet

    def isFeasible(self, item, limit, k, solTorque, solItems, cfg): # check constraints

        if item.To != self.Dests[k]:
            return False

        if self.PCV + item.V > self.V * limit:
            return False
        
        if solItems[item.ID] > -1: # if item is alocated in some pallet
            return False
        
        deltaTau = float(item.W) * float(self.D)
        if abs(solTorque.value + deltaTau) > cfg.maxTorque:
            return False

        return True
 
def loadPallets(cfg):
    """
    Load pallets attributes based on aircraft size
    """
    fname = f"./params/{cfg.size}.txt"
      
    reader = open(fname,"r")
    lines = reader.readlines()    
    pallets = []
    id = 0
    cfg.maxD = 0
    try:
        for line in lines:
            cols = line.split()
            d = float(cols[0])
            v = float(cols[1])
            w = float(cols[2])
            pallets.append( Pallet(id, d, v, w, cfg.numNodes) )
            id += 1

            if d > cfg.maxD:
                cfg.maxD = d
    finally:
        reader.close()    
    return pallets
        
def fillPallet(pallet, items, limit, k, solTorque, solItems, cfg):
    for item in items:
        if pallet.isFeasible(item, limit, k, solTorque, solItems, cfg):
            pallet.putItem(item, solTorque, solItems)
    return pallet

def palletsEnqueue(palletsQueue, p, items, limit, k, solTorque, solItems, cfg):
    palletsQueue.put( fillPallet(p, items, limit, k, solTorque, solItems, cfg) )

def shimsEnqueue( palletsQueue,    p, items, limit, k, solTorque, solItems, cfg, surplus):
    palletsQueue.put( getBestShims(p, items, limit, k, solTorque, solItems, cfg, surplus))    


def Solve(pallets, items, cfg, k, limit, secBreak): # items include kept on board

    serial = False
    # serial = True

    if serial:
        print(f"\nSerial Shims for ACLP+RPDP")
    else:
        print(f"\nParallel Shims for ACLP+RPDP")


    # surplus = 2. - limit
    surplus = math.exp(-limit) + 0.9
    print(f"surplus: {surplus:.2f}")

    numItems   = len(items)
    numPallets = len(pallets)

    solTorque = mp.Value('d') # solution global torque to be shared and changed by all pallets concurrently
    solTorque.value = 0.0

    solItems = mp.Array('i', range(numItems))
    for i, _ in enumerate(solItems):
        solItems[i] = -1 # not alocated to any pallet

    procs = [None for _ in pallets]
    palletsQueue = mp.Queue()

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

    # ------- serial greedy pallets --------
    # for i, p in enumerate(pallets):
    #     pallets[i] = fillPallet(p, items, limit, k, solTorque, solItems, cfg)

    # ------- parallel greedy pallets --------
    for i, p in enumerate(pallets):
        procs[i] = mp.Process( target=palletsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, solItems, cfg ) )
        
    for i, proc in enumerate(procs):
        time.sleep(0.001)
        proc.start()
    
    for i, _ in enumerate(procs):
        pallets[i] = palletsQueue.get( timeout = secBreak )

    if serial: # ------- serial shims --------
        for i, p in enumerate(pallets):
            bestShims = getBestShims(p, items, limit, k, solTorque, solItems, cfg, surplus)
            for item in bestShims.Items:
                if item != None:
                    pallets[i].putItem(item, solTorque, solItems)

    else: #----- parallel shims -----
        for i, p in enumerate(pallets):
            procs[i] = mp.Process( target=shimsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, solItems, cfg, surplus ) )

        for i, proc in enumerate(procs):
            time.sleep(0.001)
            proc.start()

        for i, _ in enumerate(procs):
            bestShims = palletsQueue.get( timeout = secBreak )
            for item in bestShims.Items:
                if item != None:
                    pallets[i].putItem(item, solTorque, solItems)

    # --- mount solution matrix
    Z = np.zeros((numPallets,numItems))
    for j, i in enumerate(solItems):
        if i > -1: # alocated to some pallet
            Z[i][j] = 1

    return Z

if __name__ == "__main__":

    print("----- Please execute module main -----")
