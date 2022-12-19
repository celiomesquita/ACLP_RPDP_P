# import methods as mno
import numpy as np
import multiprocessing as mp
import time
import math
# import knapsack as kp

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

    def isFeasible(self, item, k, solTorque, cfg, lock): # check constraints

        if item.To != self.Pallet.Dests[k]:
            return False

        if self.Pallet.PCV + self.SCV + item.V > self.Pallet.V:
            return False

        deltaTau = float(item.W) * float(self.Pallet.D)
        ret = True
        with lock:
            if abs(solTorque.value + self.SCT + deltaTau) > cfg.maxTorque:
                ret = False

        return ret        

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, k, solTorque, solItems, cfg, lock, surplus):

    maxVol = pallet.V * surplus

    vol = 0.
    whip = []
    
    with lock:
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
    whip.sort(key=lambda x: abs(x.V), reverse=True)

    for whipPos, item in enumerate(whip):
        newShims = True
        for sh in Set:
            if sh.isFeasible(item, k, solTorque, cfg, lock):
                sh.putItem(item, whipPos)
                newShims = False
                break

        if newShims:
            sh = Shims(pallet, len(whip)) # create a new Shim
            if sh.isFeasible(item, k, solTorque, cfg, lock):
                sh.putItem(item, whipPos)
            Set.append(sh)
                
    bestScore = 0
    bestIndex = 0
    for i, shims in enumerate(Set):
        if shims.SCS > bestScore:
            bestScore = shims.SCS
            bestIndex = i

    for item in Set[bestIndex].Items:
        if item != None:
            pallet.putItem(item, solTorque, solItems, lock)


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

    def putItem(self, item, solTorque, solItems, lock): # put an item in this pallet

        self.PCW += item.W
        self.PCV += item.V
        self.PCS += item.S

        with lock:
            solTorque.value += float(item.W) * float(self.D)
            solItems[item.ID] = self.ID # mark item as alocated to this pallet

    def isFeasible(self, item, limit, k, solTorque, solItems, cfg, lock): # check constraints

        if item.To != self.Dests[k]:
            return False

        if self.PCV + item.V > self.V * limit:
            return False

        deltaTau = float(item.W) * float(self.D)
        ret = True

        with lock:

            if solItems[item.ID] > -1: # if item is alocated in some pallet
                ret =  False
            
            if abs(solTorque.value + deltaTau) > cfg.maxTorque:
                ret =  False

        return ret
 
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
        
def fillPallet(pallet, items, k, solTorque, solItems, cfg, lock, limit):
    for item in items:
        if pallet.isFeasible(item, limit, k, solTorque, solItems, cfg, lock):
            pallet.putItem(item, solTorque, solItems, lock)

def Solve(pallets, items, cfg, k, limit, secBreak, mode, solTorque): # items include kept on board

    if mode == "p":
        print(f"\nParallel Shims for ACLP+RPDP")
    else:
        print(f"\nSerial Shims for ACLP+RPDP")

    # surplus = math.exp(-limit) + 0.9
    # surplus = 1. + 2. * (1. - limit)
    surplus = (2. - limit)

    print(f"surplus: {surplus:.2f}")

    numItems   = len(items)
    numPallets = len(pallets)

    # solTorque = mp.Value('d') # solution global torque to be shared and changed by all pallets concurrently
    # solTorque.value = 0.0

    solItems = mp.Array('i', range(numItems))
    for j, _ in enumerate(solItems):
        solItems[j] = -1 # not alocated to any pallet

    lock  = mp.Lock()

    for j, item in enumerate(items):
        items[j].ID = j
        if item.P > -1: # if alocated to a pallet
            for i, p in enumerate(pallets):
                if item.P == p.ID: # in case there is some consolidated among the items
                    pallets[i].putItem(item, solTorque, solItems, lock)

    # item.P = -1 if an item, -2 if a consollidated, or pallet ID.

    procs = [None for _ in pallets] # each pallets has its own process

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

    if mode == "p":
        # parallel greedy phase
        for i, _ in enumerate(procs):
            procs[i] = mp.Process( target=fillPallet, args=( pallets[i], items, k, solTorque, solItems, cfg, lock, limit) )
            time.sleep(0.001)
            procs[i].start()
        
        for proc in procs:
            proc.join()

        # parallel shims phase
        for i, _ in enumerate(procs):
            procs[i] = mp.Process( target=getBestShims, args=( pallets[i], items, k, solTorque, solItems, cfg, lock, surplus) )
            procs[i].start()
        
        start = time.time()
        while time.time() - start <= secBreak:
            if not any(p.is_alive() for p in procs):
                # All the processes are done, break now.
                break
        else:
            # We only enter this if we didn't 'break' above.
            print("timed out, killing all processes")
            for p in procs:
                p.terminate()
                p.join()

    else: # serial
        for i, _ in enumerate(pallets):
            fillPallet(  pallets[i], items, k, solTorque, solItems, cfg, lock, limit) 
            getBestShims(pallets[i], items, k, solTorque, solItems, cfg, lock, surplus)
               

    # --- mount solution matrix
    Z = np.zeros((numPallets,numItems))
    for j, i in enumerate(solItems):
        if i > -1: # alocated to some pallet
            Z[i][j] = 1

    return Z

if __name__ == "__main__":

    print("----- Please execute module main -----")
