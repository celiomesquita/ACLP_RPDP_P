
import methods as mno
import numpy as np
import multiprocessing as mp
import time

# A Shims fits in a pallet slack
class Shims(object):
    def __init__(self, pallet):
        self.SCW     = 0   # shims current weight (must fit the slack)
        self.SCV     = 0.0 # shims current volume (must fit the slack)
        self.SCS     = 0   # shims current score
        self.SCT     = 0.0 # shims current torque
        self.InSol = [False for _ in items] # True if an item is in the shims
        self.Pallet = pallet # this shims is one of the possible for this pallet
        self.Items = []

    def putItem(self, item, k, solTorque, cfg):
     

        deltaTau = float(item.W) * float(self.Pallet.D)

        if  item.To == self.Pallet.Dests[k]                      and \
            self.Pallet.PCW + self.SCW + item.W <= self.Pallet.W and \
            self.Pallet.PCV + self.SCV + item.V <= self.Pallet.V and \
            not self.Pallet.InSol[item.ID]                       and \
            not self.InSol[item.ID]                              and \
            abs(solTorque.value + deltaTau) <= cfg.maxTorque:
            
            self.SCW += item.W
            self.SCV += item.V
            self.InSol[item.ID] = True
            self.SCS += item.S

            self.Items.append(item)

            return True
        
        return False

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, limit, items, k, solTorque, cfg):

    # create the first shim
    sh = Shims(pallet)
    Set = [sh]

    maxVol = pallet.PCV * (4. - 3*limit)

    vol = 0.
    whip = []
    for item in items:
        if not pallet.InSol[item.ID]:
            vol += item.V
            whip.append(item)
            if vol > maxVol:
                break

    # First Fit Decrease - equivalente ao KP, mas mais rápido
    included = 0
    for item in whip:
        NotIncluded = True
        for sh in Set:
            if sh.putItem(item, k, solTorque, cfg):
                NotIncluded = False
                included += 1
                break

        if NotIncluded:
            sh = Shims(pallet) # create a new Shim
            if sh.putItem(item, k, solTorque, cfg): # insert the item in the new Shim
                Set.append(sh)
                # included += 1

    bestScore = 0
    bestIndex = 0
    for i, shims in enumerate(Set):
        if shims.SCS > bestScore:
            bestScore = shims.SCS
            bestIndex = i

    print(len(whip), len(Set), bestIndex, included)

    return Set[bestIndex] 


class Pallet(object):
    def __init__(self, id, d, v, w, numNodes, items):
        self.ID = id
        self.D  = d  # centroid distance to CG
        self.V  = v  # volume limit
        self.W  = w  # weight limit
        self.Dests = np.full(numNodes, -1)
        self.InSol = [False for _ in items] # True if an item is in this pallet
        self.PCW = 0 # pallet current weight
        self.PCV = 0.
        self.S = 0.

    def putItem(self, item, limit, k, solTorque, cfg):

        deltaTau = float(item.W) * float(self.D)

        if  item.To == self.Dests[k] and \
            self.PCW + item.W <= self.W and \
            self.PCV + item.V <= self.V * limit and \
            not self.InSol[item.ID] and \
            abs(solTorque.value + deltaTau) <= cfg.maxTorque:
            
            self.PCW += item.W
            self.PCV += item.V
            self.InSol[item.ID] = True

            # with solTorque.get_lock():
            solTorque.value += deltaTau

            self.S += item.S


def loadPallets(cfg, items):

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
            pallets.append( Pallet(id, d, v, w, cfg.numNodes, items) )
            id += 1

            if d > cfg.maxD:
                cfg.maxD = d
    finally:
        reader.close()    
    return pallets
  
        
def fillPallet(p, items, limit, k, solTorque, cfg):
    for item in items:
        p.putItem(item, limit, k, solTorque, cfg)
    return p

def palletsEnqueue(palletsQueue, p, items, limit, k, solTorque, cfg):
    palletsQueue.put( fillPallet(p, items, limit, k, solTorque, cfg) )

def printPallets(pallets, cfg, solTorque, message):
    print(message)
    pallets.sort(key=lambda x: abs(x.ID))
    solScore  = 0.
    for p in pallets:
        solScore += p.S
        print(f"{p.ID}\t{p.PCW}\t{p.PCV:.2f}")
    print(f"{solScore}\t{solTorque.value/cfg.maxTorque:.2f}")

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

    surplus = "data20"
    # surplus = "data50"
    # surplus = "data100"

    items = mno.loadNodeItems(scenario, inst, node, unattended, surplus)

    pallets = loadPallets(cfg, items)

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

    limit = 0.5

    solTorque = mp.Value('d') # solution global torque to be shared and changed by all pallets concurrently

    # for i, p in enumerate(pallets):
    #     pallets[i] = fillPallet(p, items, limit, k, solTorque, cfg)


    procs = [None for _ in pallets]
    palletsQueue = mp.Queue()

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False) 

    for i, p in enumerate(pallets):
        procs[i] = mp.Process( target=palletsEnqueue,args=( palletsQueue, p, items, limit, k, solTorque, cfg ) )
        
    for i, proc in enumerate(procs):
        # closer to CG pallets start to be solved first
        time.sleep(abs(pallets[i].D)/1000.)

        proc.start()
    
    for i, _ in enumerate(procs):
        pallets[i] = palletsQueue.get( timeout = 0.7 )

    printPallets(pallets, cfg, solTorque, f"\n---Greedy solution---{limit}---{len(items)} items")


    counter = 0
    # limit = 1.
    for i, pallet in enumerate(pallets):
        shims = getBestShims(pallet, limit, items, k, solTorque, cfg)

        for item in shims.Items:
            pallets[i].putItem(item, limit, k, solTorque, cfg)
            counter += 1

    printPallets(pallets, cfg, solTorque, f"\n---Shims solution---{counter} items")
