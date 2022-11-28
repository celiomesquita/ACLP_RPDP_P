import methods as mno
import multiprocessing as mp
import numpy as np

# A Shim is a thin and often tapered or wedged piece of material, used to fill small gaps or spaces between objects.
# Set are typically used in order to support, adjust for better fit, or provide a level surface.
# Set may also be used as spacers to fill gaps between parts subject to wear.

# Shims fit in a pallet slack
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

        # if ce.Item.ID > len(self.Included)-1:
        #     return False

        if self.Included[ce.Item.ID] > 0 or sol.Included[ce.Item.ID] > 0:
            return False # this item was already inserted.
          
        # if ce.Item.To != ce.Pallet.Dests[k]:
        #     return False #item and pallet destinations are different. Equation 21
        
        # Pallet Acumulated Weight
        # if sol.PAW[ce.Pallet.ID] + self.W + ce.Item.W > ce.Pallet.W:
        #     return False #this item weight would exceed pallet weight limit. Equation 17
        
        # Pallet Acumulated Volume
        if sol.PAV[ce.Pallet.ID] + self.V + ce.Item.V > ce.Pallet.V*1.03:
            return False #this item volume would exceed pallet volumetric limit. Equation 18
        
        # newTorque = self.T + sol.T + ce.Torque
        # if abs(newTorque) > maxTorque:
        #     return False
        
        return True
        


# solve a Subset Selection Problem for this pallet, by selecting
# the best shim and including its edges in solution.
def getBestShims(p, notInSol, sol, limit, numItems, maxTorque, k):

    # create the first shim
    sh = Shim(numItems)
    Set = [sh]

    # calculate the k2 length
    # it is calculated by a volume estimation of 1 + 3*slack volume percent
    # of the partial solution's volume.
    # Only a small portion of the notInSol vector is assessed to find the a local best Shim.

    vol = sol.PAV[p.ID] # current pallet volume
    maxVol = vol * (4. - 3*limit)
   
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
            if sh.shimIsFeasible(oe, sol, maxTorque, k): # try to insert the edge "oe" in the new Shim
                sh.AppendEdge(oe)
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
      


def shimsEnqueue( outQueue,          p, notInSol, sol, limit, numItems, maxTorque, k):
    outQueue.put( getBestShims(p, notInSol, sol, limit, numItems, maxTorque, k) )

# parallel Shims based on different limits
def Solve(pallets, items, cfg, k, limit, secBreak): # items include kept on board

    print(f"\nParallel Shims for ACLP+RPDP")

    numItems   = len(items)
    numPallets = len(pallets)

    edges = mno.mountEdges(pallets, items, cfg)

    sol = mno.Solution(edges, pallets, items, limit, cfg, k)

    notInSol = [ [] for _ in range(len(pallets)) ]

    procs = [None for _ in pallets]
    outQueue = mp.Queue()

    for i, p in enumerate(pallets):

        notInSol[i] = [e for e in sol.Edges if e.Pallet.ID == p.ID and not e.InSol  ]

        procs[i] = mp.Process( target=shimsEnqueue,args=( outQueue, p, notInSol[i], sol, limit, numItems, cfg.maxTorque, k  ) )
        
    for p in procs:
        p.start()
    
    for _ in procs:
        BestShims = outQueue.get( timeout=secBreak )
        for e in BestShims:
            sol.putInSol(e)

    # local search
    counter = 0
    for e in sol.Edges:
        if not e.InSol and sol.isFeasible(e, 1.05, cfg, k):
            sol.putInSol(e)
            counter += 1
    print(f"{counter} extra edges put in the best solution")

    return mno.getSolMatrix(sol.Edges, numPallets, numItems)
        
if __name__ == "__main__":

    print("----- Please execute module main -----")
