
import methods as mno
import numpy as np
import multiprocessing as mp
import time
import ctypes as ct

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
def getBestShims(pallet, items, limit, k, solTorque, X, cfg):

    # create the first shim
    sh = Shims(pallet, len(items))
    Set = [sh]

    whipLen = pallet.V * (3. - 2*limit) # whip length

    vol = 0.
    whip = []
    
    for item in items:
        if X[pallet.ID][item.ID] == 0:
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

    def putItem(self, item, solTorque, X): # put an item in this pallet

        self.PCW += item.W
        self.PCV += item.V
        self.PCS += item.S

        with solTorque.get_lock():
            solTorque.value += float(item.W) * float(self.D)

        X[self.ID][item.ID] = 1

    def isFeasible(self, item, limit, k, solTorque, X, cfg): # check constraints

        if item.To != self.Dests[k]:
            return False

        if self.PCV + item.V > self.V * limit:
            return False

        if X[self.ID][item.ID] == 1:
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
        
def fillPallet(p, items, limit, k, solTorque, X, cfg):
    for item in items:
        if p.isFeasible(item, limit, k, solTorque, X, cfg):
            p.putItem(item, solTorque, X)
    return p

def palletsEnqueue(palletsQueue, p, items, limit, k, solTorque, X, cfg):
    palletsQueue.put( fillPallet(p, items, limit, k, solTorque, X, cfg) )

def shimsEnqueue( palletsQueue,    p, items, limit, k, solTorque, X, cfg):
    palletsQueue.put( getBestShims(p, items, limit, k, solTorque, X, cfg))    

def printPallets(pallets, cfg, solTorque, message):
    print(message)
    print("Pallet\tWeight\t%\tVolume\t%")
    pallets.sort(key=lambda x: abs(x.ID))
    solCScore  = 0.
    solWeight  = 0.
    solVolume  = 0.
    solCWeight  = 0.
    solCVolume  = 0.    
    for p in pallets:
        solWeight += p.W
        solVolume += p.V

        solCScore  += p.PCS
        solCWeight += p.PCW
        solCVolume += p.PCV

        print(f"{p.ID}\t{p.PCW}\t{p.PCW/p.W:.2f}\t{p.PCV:.2f}\t{p.PCV/p.V:.2f}")
    print("Score\tTorque\tWeight\tVolume")
    print(f"{solCScore}\t{solTorque.value/cfg.maxTorque:.2f}\t{solCWeight/solWeight:.2f}\t{solCVolume/solVolume:.2f}")
    print()

if __name__ == "__main__":
    

    scenario = 1
    inst = 1

    cfg = mno.Config(scenario)

    dists = mno.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value

    tours = mno.getTours(cfg.numNodes-1, costs, 0.25)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 0 # the base

    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    node = tour.nodes[k]

    # surplus = "data20"
    surplus = "data50"
    # surplus = "data100"

    # loads items and sort by the Score/Volume ratio
    items = mno.loadNodeItems(scenario, inst, node, unattended, surplus)

    pallets = loadPallets(cfg)

    mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

    # pallets capacity
    cfg.weiCap = 0
    cfg.volCap = 0
    for p in pallets:
        cfg.weiCap += p.W
        cfg.volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if cfg.weiCap > cfg.payload:
        cfg.weiCap = cfg.payload 

    limit = 0.5 # greedy limit

    numPallets = len(pallets)
    numItems   = len(items)

    solTorque = mp.Value('d') # solution global torque to be shared and changed by all pallets concurrently
    solTorque.value = 0.0

    arr = mp.Array('i', range(numPallets*numItems))
    arr = np.frombuffer(arr.get_obj(), ct.c_int)
    X = arr.reshape((numPallets,numItems))

    # for i, p in enumerate(pallets):
    #     pallets[i] = fillPallet(p, items, limit, k, solTorque, X, cfg)
    # printPallets(pallets, cfg, solTorque, f"\n---Greedy solution---{limit}---{len(items)} items")

    # ------- parallel pallets --------
    procs = [None for _ in pallets]
    palletsQueue = mp.Queue()

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False) 

    for i, p in enumerate(pallets):
        procs[i] = mp.Process( target=palletsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, X, cfg ) )
        
    for i, proc in enumerate(procs):
        # closer to CG pallets start to be solved first
        time.sleep(abs(pallets[i].D)/1000.)

        proc.start()
    
    for i, _ in enumerate(procs):
        pallets[i] = palletsQueue.get( timeout = 0.7 )

    printPallets(pallets, cfg, solTorque, f"\n---Greedy solution---{limit}---{len(items)} items")

   
    # multiprocessing shims solution
    for i, p in enumerate(pallets):
        procs[i] = mp.Process( target=shimsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, X, cfg ) )
        
    for i, proc in enumerate(procs):
        # closer to CG pallets start to be solved first
        time.sleep(abs(pallets[i].D)/1000.)        
        proc.start()
    
    for i, _ in enumerate(procs):
        shims = palletsQueue.get( timeout = 0.7 )
        for item in items:
            if shims.InSol[item.ID]:
                pallets[i].putItem(item, solTorque, X)


    # for i, pallet in enumerate(pallets):
    #     shims = getBestShims(pallet, items, limit, k, solTorque, X, cfg)
    #     for item in items:
    #         if shims.InSol[item.ID]:
    #             pallets[i].putItem(item, solTorque)


    printPallets(pallets, cfg, solTorque, f"\n---Shims solution---")
