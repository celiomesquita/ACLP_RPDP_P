import methods as mno
import numpy as np

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

    def AppendEdge(self, e):
        self.W += e.Item.W
        self.V += e.Item.V
        self.S += e.Item.S
        self.T += float(e.Item.W) * e.Pallet.D
        self.Edges.append(e)
        self.Included[e.Item.ID] = 1        

    def shimIsFeasible(self, ce, sol, maxTorque, k ):

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
        
        newTorque = self.T + sol.T + ce.Torque
        if abs(newTorque) > maxTorque:
            return False
        
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

    vol = sol.PAV[p.ID] # current pallet ocupation
    maxVol = vol * (2. - limit)
   
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


def Compute(edges, pallets, items, limit, cfg, k) :

    sol = mno.Solution(edges, pallets, items, limit, cfg, k)

    #edges not in sol groupped by pallets
    notInSol = [ [] for _ in range(len(pallets)) ]

    for p in (pallets):
        notInSol[p.ID] = [e for e in sol.Edges if e.Pallet.ID == p.ID and not e.InSol  ]

	# pallets closer to the CG are completed first
    pallets.sort(key=lambda x: abs(x.D), reverse=False)

	# get the best shim that fills the slack
    for p in (pallets):

		# get the best shim of edges             greedy limit
        sedges = getBestShim(p, notInSol[p.ID], sol, limit, len(items), cfg.maxTorque, k)

        # move the best shim of edges to solution
        for be in sedges:
            sol.putInSol(be)
            notInSol[p.ID].remove(be)

    counter = 0
    # local search: Maybe still exists any edge that fit in solution
    for nis in notInSol: # for each row
        for ce in nis:
            if sol.isFeasible(ce, 1.0, cfg, k):
                sol.putInSol(ce)
                counter += 1
    print(f"{counter} edges included by the local search\n")

    return sol


def Solve(pallets, items, cfg, k, limit): # items include kept on board

    print(f"\nShims for ACLP+RPDP")

    numItems   = len(items)
    numPallets = len(pallets)

    edges = mno.mountEdges(pallets, items, cfg)

    sol = Compute(edges, pallets, items, limit, cfg, k)

    return mno.getSolMatrix(sol.Edges, numPallets, numItems)
        
        
if __name__ == "__main__":


    # mno.DATA = "data20"
    mno.DATA = "data50"
    # mno.DATA = "data100"
  
    method = "Shims"

    scenario = 1

    cfg = mno.Config(scenario)

    if scenario == 1:
        instances = [1,2,3,4,5,6,7]
        # instances = [1]
    if scenario == 2:
        instances = [1,2,3,4,5,6,7]
    if scenario == 3:
        instances = [1,2,3,4,5,6,7]
    if scenario == 4:
        instances = [1,2,3,4,5,6,7] 
    if scenario == 5:
        instances = [1,2,3,4,5]     
    if scenario == 6:
        instances = [1,2,3]                                          

    dists = mno.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]
    
    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value

    pallets = mno.loadPallets(cfg)

    # pallets capacity
    cfg.weiCap = 0
    cfg.volCap = 0
    for p in pallets:
        cfg.weiCap += p.W
        cfg.volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if cfg.weiCap > cfg.payload:
        cfg.weiCap = cfg.payload   

    tours = mno.getTours(cfg.numNodes, costs, 0.25)

    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 0 # the base

    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    node = tour.nodes[k]
    print(node.ICAO)

    accumLim = 0.
    denom = 50

    for instance in instances:

        # load items parameters from this node and problem instance, that go to unnatended
        items = mno.loadNodeItems(scenario, instance, node, unattended)

        numItems = len(items)

        print(f"{numItems} items")

        mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

        print("Dests: ",end="")
        for p in pallets:
            print(f"{mno.CITIES[p.Dests[k]]} ", end='')
        print()


        scoreMax = 0.
        bestLim  = 0.        

        for v in range(denom):

            limit = (v+1) / float(denom)

            E = Solve(pallets, items, cfg, 0, limit)

            consJK = [
                        [ mno.Item(-1, -2, 0, 0, 0., -1, -1)
                        for _ in tour.nodes ]
                        for _ in pallets # a consolidated for each pallet
                    ] 

            # print the solution for this node
            if len(E) > 0:

                consNodeT = [None for _ in pallets]

                pallets.sort(key=lambda x: x.ID)  

                for j, p in enumerate(pallets):

                    consJK[j][k].ID  = j+numItems
                    consJK[j][k].Frm = node.ID
                    consJK[j][k].To  = p.Dests[k]

                    for i in np.arange(numItems):

                        if E[j][i] == 1:

                            consJK[j][k].W += items[i].W
                            consJK[j][k].V += items[i].V
                            consJK[j][k].S += items[i].S

                    consNodeT[j] = consJK[j][k]

                sNodeAccum = 0.
                wNodeAccum = 0.
                vNodeAccum = 0.
                tau = 0.
                sol = ""

                for i, p in enumerate(pallets):
                    sNodeAccum += float(consJK[i][k].S)
                    # wNodeAccum += float(consJK[i][k].W)
                    # vNodeAccum += float(consJK[i][k].V)
                    # tau        += float(consJK[i][k].W) * pallets[i].D

                # epsilom = tau/cfg.maxTorque

                # Heuristic = (1-abs(epsilom)) * sNodeAccum / vNodeAccum
                # heuSum += Heuristic

                # if Heuristic > heuMax:
                #     heuMax = Heuristic
                    # bestLim = limit


                if sNodeAccum > scoreMax:
                    scoreMax = sNodeAccum
                    bestLim = limit                    

                # sol += f"Score: {sNodeAccum}\t"
                # sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
                # sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
                # sol += f"Torque: {epsilom:.2f}\n"
                # sol += f"Heuristic: {Heuristic:.0f}\n" 
                # print(sol)

        accumLim += bestLim

    accumLim /= float(len(instances))

    print(f"Best limit = {accumLim:.2f}")


    # print("----- Please execute module main_test -----")
