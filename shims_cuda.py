import methods
import numpy as np
import math
from numba import cuda, jit
from timeit import default_timer as timer


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

    def AppendEdge(self, e):
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
        if sol.PAW[ce.Pallet.ID] + self.W + ce.Item.W > ce.Pallet.W:
            return False #this item weight would exceed pallet weight limit. Equation 17
        
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
@jit
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
                sh.AppendEdge(oe)
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

@jit
def Compute(edges, pallets, items, limit, cfg, k) :

    sol = methods.Solution(edges, pallets, items, limit, cfg, k)

    notInSol = [ [] for _ in range(len(pallets))]

    for p in (pallets):
        notInSol[p.ID] = [edge for edge in sol.Nbhood if edge.Pallet.ID == p.ID  ]

	# pallets closer to the CG are completed first
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

	# get the best shim that fills the slack
    for p in (pallets):

		# get shim sh edges                      greedy limit
        sedges = getBestShim(p, notInSol[p.ID], sol, limit, len(items), cfg.aircraft.maxTorque, k)

        # move shim sh edges to solution
        for be in sedges:
            sol.AppendEdge(be)
            notInSol[p.ID].remove(be)

    for nis in notInSol: # for each row in the matrix +slacks[ce.Pallet.ID]
        for ce in nis:
            if sol.isFeasible(ce, 1.0, cfg, k):
                sol.AppendEdge(ce)
    return sol

def Solve(pallets, items, cfg, k): # items include kept on board

    print(f"\nShims_cuda, a new GPU Heuristic for ACLP+RPDP")

    edges = methods.mountEdges(pallets, items, cfg, k)

    numItems   = len(items)
    numPallets = len(pallets)

    # as greater is the number of edges, closest to 1 is the limit

    # NUM = 35
    NUM = 2200

    if methods.DATA == "data50":
        NUM *= 2
    if methods.DATA == "data100":
        NUM *= 3

    step = math.floor(NUM/methods.NCPU)

    num = []

    last1 = NUM
    last2 = NUM

    for i in range(methods.NCPU):
        if i % 2 == 0:
            num.append(last1)
            last1 += step
        else:
            last2 -= step
            num.append(last2)
    
    lims = []
    for i in range(methods.NCPU):
        limit = 1.0 - num[i]/float(numItems*numPallets)
        lims.append(limit)


    # Set the number of threads in a block
    threadsperblock = 32 

    # Calculate the number of thread blocks in the grid
    blockspergrid = (numItems*numPallets*len(edges) + (threadsperblock - 1)) # threadsperblock

    Compute[blockspergrid, threadsperblock](edges, pallets, items, limit, cfg, k)
    sols = cuda.profile_stop()

    best = 0
    edges = []
    for sol in sols:
        if sol.Heuristic > best:
            best = sol.Heuristic
            edges = methods.edges_copy(sol.Edges)

    # decision matrix for which items will be put in which pallet
    X = np.zeros((numPallets,numItems)) # needs 2 parenthesis
    for e in edges:
        X[e.Pallet.ID][e.Item.ID] = 1

    return X
        
if __name__ == "__main__":

    print("----- Please execute module main_test -----")