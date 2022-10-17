import methods
import numpy as np
# import math
# import copy
import multiprocessing as mp
import time

NCPU = 8

# A Shim is a thin and often tapered or wedged piece of material, used to fill small gaps or spaces between objects.
# Set are typically used in order to support, adjust for better fit, or provide a level surface.
# Set may also be used as spacers to fill gaps between parts subject to wear.

# Shim fits in a slack in a pallet
class Shim(object):
    def __init__(self, numItems):
        self.W        = 0   # total weight of the shim set (must fit the slack)
        self.V        = 0.0 # total volume of the shim set (must fit the slack)
        self.S        = 0   # score of the shim set
        self.T        = 0.0 # torque of the shim set
        self.Edges    = []
        self.Included = [ 0  for _ in np.arange(numItems) ]   # items included in shim set edges
        self.numItems = numItems

    def includeEdge(self, e):
        self.W += e.Item.W
        self.V += e.Item.V
        self.S += e.Item.S
        self.T += float(e.Item.W) * e.Pallet.D
        self.Edges.append(e)
        self.Included[e.Item.ID] = 1        

    def shimIsFeasible(self, ce, sol, maxTorque, k):

        if ce.Item.ID > len(self.Included)-1:
            return False

        if self.Included[ce.Item.ID] > 0 or sol.Included[ce.Item.ID] > 0:
            return False # this item was already inserted.
          
        if ce.Item.To != ce.Pallet.Dests[k]:
            return False #item and pallet destinations are different. Equation 21
        
        # Pallet Acumulated Weight
        # if sol.PAW[ce.Pallet.ID] + self.W + ce.Item.W > ce.Pallet.W:
        #     return False #this item weight would exceed pallet weight limit. Equation 17
        
        # Pallet Acumulated Volume
        if sol.PAV[ce.Pallet.ID] + self.V + ce.Item.V > ce.Pallet.V:
            return False #this item volume would exceed pallet volumetric limit. Equation 18
        
        # if this inclusion increases torque and ....
        currentTorque = self.T + sol.T
        newTorque = currentTorque + ce.Torque
        if  abs(currentTorque) < abs(newTorque):
            # ... it is aft
            if newTorque > maxTorque:
                return False #this item/pallet torque would extend the CG shift beyond backward limit. Equation 15
            # ... it is fwd
            if newTorque < -maxTorque:
                return False #this item/pallet torque would extend the CG shift beyond forward limit. Equation 10

        return True
        


# solve a Subset Selection Problem for this pallet, by selecting
# the best shim and including its edges in solution.
def getBestShim(p, notInSol, sol, limit, numItems, maxTorque, k):

    # create the first shim
    sh = Shim(numItems)
    Set = [sh]

    # calculate the k2 length
    # it is calculated by a volume estimation of 1 + 3*slack volume percent
    # of the partial solution's volume.
    # Only a small portion of the notInSol vector is assessed to find the a local best Shim.

    slack = 1 - limit # greedy limit

    vol = sol.PAV[p.ID] # current pallet ocupation
    maxVol = vol * (1.0 + 3.0*slack)
   
    k2 = 0
    for e in notInSol:
        vol += e.Item.V
        k2 += 1
        if vol > maxVol:
            break
    
    outter = notInSol[:k2-1]

    outter.sort(key=lambda x: x.Item.V, reverse=True)

    # First Fit Decrease - equivalente ao KP, mas mais rápido
    for i, oe in enumerate(outter):
        for sh in Set:
            if sh.shimIsFeasible(oe, sol, maxTorque, k):
                sh.includeEdge(oe)
                break
        else:
            sh = Shim(numItems) # create a new Shim
            Set.append(sh)

    # all Set of edges are feasible, but one has a better score
    if len(Set) > 0 :
        #look for the best weight and volume Set
        maxW = 0
        maxV = 0.0
        bsW = 0
        bsV = 0
        bestScoreW = 0
        bestScoreV = 0

        for i, sh in enumerate(Set):
            #max weight shim, usefull if weight is maximized
            if maxW < sh.W :
                maxW = sh.W
                bsW = i # best score weight index
                bestScoreW = sh.S
            
            #max volume shim, useful volume is maximized
            if maxV < sh.V  :
                maxV = sh.V
                bsV = i# best score volume index
                bestScoreV = sh.S
            
            #best score shim index
            bsi = bsW
            if bestScoreW < bestScoreV :
                bsi = bsV
            
        return Set[bsi].Edges #this Set enters the solution
        
    return []

def Compute(edges, pallets, items, limit, cfg, k) :

    sol = methods.Solution(edges, pallets, items, limit, cfg, k)

    notInSol = [ [] for _ in range(len(pallets))]

    for p in (pallets):
        notInSol[p.ID] = [e for e in sol.Edges if not e.InSol and e.Pallet.ID == p.ID  ]

	# pallets closer to the CG are completed first
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

	# get the best shim that fills the slack
    for p in (pallets):

		# get shim sh edges                      greedy limit
        sedges = getBestShim(p, notInSol[p.ID], sol, limit, len(items), cfg.aircraft.maxTorque, k)

        # move shim sh edges to solution
        for be in sedges:
            sol.includeEdge(be)
            notInSol[p.ID].remove(be)

    for nis in notInSol:
        for ce in nis:
            if sol.isFeasible(ce, 1.0, cfg, k):
                sol.includeEdge(ce)

    return sol

def comp_queue(    pallets, numItems, sol, limit, cfg, k, q):
    q.put(Compute(pallets, numItems, sol, limit, cfg, k))

def Solve(pallets, items, cfg, k): # items include kept on board

    t0 = time.perf_counter()

    print(f"\nShims, a new Heuristic for ACLP+RPDP")

    edges = methods.mountEdges(pallets, items, cfg, k)

    numItems   = len(items)
    numPallets = len(pallets)

    # as greater is the number of edges, closest to 1 is the limit
    num = [600+100*(i+1) for i in range(NCPU)]
    if methods.DATA == "data50":
        num = [1600+100*(i+1) for i in range(NCPU)]
    if methods.DATA == "data100":
        num = [2600+100*(i+1) for i in range(NCPU)]

    lims = []
    for i in range(NCPU):
        lims.append(1.0 - num[i]/float(numItems*numPallets))

    procs = [None for _ in range(NCPU)]
    out_queue = mp.Queue()

    for i in range(NCPU):
        procs[i] = mp.Process(target=comp_queue,args=(edges, pallets, items, lims[i], cfg, k, out_queue))
    for p in procs:
        p.start()

    t1 = time.perf_counter()

        # wait for all computations
    sols = [out_queue.get() for _ in procs]

    t2 = time.perf_counter()

    best = 0
    edges = []
    for sol in sols:
        if sol.S > best:
            best = sol.S
            edges = sol.Edges

    # decision matrix for which items will be put in which pallet
    X = np.zeros((numPallets,numItems)) # needs 2 parenthesis
    for e in edges:
        if e.InSol == 1:
            X[e.Pallet.ID][e.Item.ID] = 1

    print(f"durations\t{t1-t0:.3f}\t{t2-t1:.3f}\n")

    return X
        
if __name__ == "__main__":

    print("----- Please execute module main_test -----")
