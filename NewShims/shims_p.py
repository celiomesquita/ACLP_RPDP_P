# import methods as mno
import numpy as np
import multiprocessing as mp
import time

# A Shims fits in a pallet slack
class Shims(object):
    def __init__(self, pallet, numItems):
        self.Pallet = pallet # this shims is one of the possibles for this pallet
        self.SCW    = 0   # shims current weight (must fit the slack)
        self.SCV    = 0.0 # shims current volume (must fit the slack)
        self.SCS    = 0   # shims current score
        self.InSol  = [0 for _ in np.arange(numItems)] # 1 if an item is in the shims
        self.SCT    = 0. # shims current torque

    def putItem(self, item):
                 
            self.SCW += item.W
            self.SCV += item.V
            self.SCS += item.S
            self.InSol[item.ID] = 1
            self.SCT += float(item.W) * float(self.Pallet.D)

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
def getBestShims(pallet, items, limit, k, solTorque, solItems, cfg):

    # create the first shim
    sh = Shims(pallet, len(items))
    Set = [sh]

    whipLen = pallet.V * (3. - 2*limit) # whip length

    vol = 0.
    whip = []
    
    for item in items:
        if solItems[item.ID] == -1: # not allocated in any pallet
            vol += item.V
            whip.append(item)
            if vol > whipLen:
                break

    # First Fit Decrease - equivalente ao KP, mas mais rápido
    # included1 = 0
    # included2 = 0
    for item in whip:
        newShims = True
        for sh in Set:
            if sh.isFeasible(item, k, solTorque, cfg):
                sh.putItem(item)
                newShims = False
                # included1 += 1
                break

        if newShims:
            sh = Shims(pallet, len(items)) # create a new Shim
            if sh.isFeasible(item, k, solTorque, cfg):
                sh.putItem(item)
                Set.append(sh)
                # included2 += 1

    bestScore = 0
    bestIndex = 0
    for i, shims in enumerate(Set):
        if shims.SCS > bestScore:
            bestScore = shims.SCS
            bestIndex = i

    # print(len(whip), len(Set), bestIndex, included1, included2)

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
        
def fillPallet(p, items, limit, k, solTorque, solItems, cfg):
    for item in items:
        if p.isFeasible(item, limit, k, solTorque, solItems, cfg):
            p.putItem(item, solTorque, solItems)
    return p

def palletsEnqueue(palletsQueue, p, items, limit, k, solTorque, solItems, cfg):
    palletsQueue.put( fillPallet(p, items, limit, k, solTorque, solItems, cfg) )

def shimsEnqueue( palletsQueue,    p, items, limit, k, solTorque, solItems, cfg):
    palletsQueue.put( getBestShims(p, items, limit, k, solTorque, solItems, cfg))    


def Solve(pallets, items, cfg, k, limit, secBreak): # items include kept on board

    print(f"\nParallel Shims for ACLP+RPDP")

    numItems   = len(items)
    numPallets = len(pallets)

    solTorque = mp.Value('d') # solution global torque to be shared and changed by all pallets concurrently
    solTorque.value = 0.0

    solItems = mp.Array('i', range(numItems))
    for i, _ in enumerate(solItems):
        solItems[i] = -1 # not alocated to any pallet

    # ------- parallel greedy pallets --------
    procs = [None for _ in pallets]
    palletsQueue = mp.Queue()

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False) 

    for i, p in enumerate(pallets):
        procs[i] = mp.Process( target=palletsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, solItems, cfg ) )
        
    for i, proc in enumerate(procs):
        time.sleep(0.001)
        proc.start()
    
    for i, _ in enumerate(procs):
        pallets[i] = palletsQueue.get( timeout = secBreak )

    # ------- serial greedy pallets --------
    # for i, p in enumerate(pallets):
    #     pallets[i] = fillPallet(p, items, limit, k, solTorque, solItems, cfg)


    # ----- parallel shims -----
    for i, p in enumerate(pallets):
        procs[i] = mp.Process( target=shimsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, solItems, cfg ) )

    for i, proc in enumerate(procs):
        # time.sleep(0.001)
        proc.start()

    for i, _ in enumerate(procs):
        shims = palletsQueue.get( timeout = secBreak )
        for j, v in enumerate(shims.InSol):
            if v == 1:
                pallets[i].putItem(items[j], solTorque, solItems)

    # ------- serial shims --------
    # for i, p in enumerate(pallets):
    #     bestShims = getBestShims(p, items, limit, k, solTorque, solItems, cfg)
    #     for j, v in enumerate(bestShims.InSol):
    #         if v == 1:
    #             pallets[i].putItem(items[j], solTorque, solItems)


    # --- mount solution matrix
    Z = np.zeros((numPallets,numItems))
    for j, i in enumerate(solItems):
        if i > -1: # alocated to some pallet
            Z[i][j] = 1

    return Z

if __name__ == "__main__":

    print("----- Please execute module main -----")
