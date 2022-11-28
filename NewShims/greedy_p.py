
import methods as mno
import numpy as np

class Edge(object):
    def __init__(self, id, pallet, item, cfg):

        self.ID      = id # index in the solution edges (it must be preserved in case of sorting)
        self.Pallet  = pallet
        self.Item    = item
        self.Torque  = float(item.W) * float(pallet.D)

        # marginal torque
        # 5100 = 15m x 340kg, maximum torque possible
        marginal = abs(self.Torque) / (cfg.maxD*340) # less than 1

        self.Heuristic = ( float(item.S) / ( 500 * float(item.V) ) ) * (1.0 - marginal)

# Edges sorted by heuristic attractiveness
def mountEdges(pallets, items, cfg):
   
    # items include kept on board, are from this node (k), and destined to unattended nodes
    m = len(pallets)
    n = len(items)

    edges = [None for _ in np.arange(m*n)]

    i = 0
    for p in pallets:           
        for it in items:
            e = Edge(i, p, it, cfg)
            edges[i] = e
            i += 1

    # greater heuristic's attractivity are included first
    edges.sort(key=lambda x: x.Heuristic, reverse=True) # best result

    # as edges have its order changed the ID must in the original order to preserve reference
    for i, e in enumerate(edges):
        edges[i].ID = i

    return edges

class Sol(object):
    def __init__(self, pallets, items, cfg):

        self.PCW = [ 0   for _ in pallets] # set of pallets current weights
        self.PCV = [ 0.0 for _ in pallets] # set of pallets current volumes
        self.T = sum(140 * p.D for p in pallets)  # solution total torque
        self.S = 0 # solution total score
        self.InSol = [False for _ in np.arange(len(pallets)*len(items))]

    def put(self, ce): # mark this edge as in solution

        self.PCW[ce.Pallet.ID] += ce.Item.W
        self.PCV[ce.Pallet.ID] += ce.Item.V
        self.S += ce.Item.S
        self.T += ce.Torque
        self.InSol[ce.ID] = True

    def isFeasible(self, ce, limit, cfg, k):

        if self.InSol[ce.ID]:
            return False

        if ce.Item.To != ce.Pallet.Dests[k]:
            return False
        
        if self.PCW[ce.Pallet.ID] + ce.Item.W > ce.Pallet.W:
             return False
        
        if self.PCV[ce.Pallet.ID] + ce.Item.V > ce.Pallet.V * limit:
            return False
        
        if abs(self.T + ce.Torque) > cfg.maxTorque:
            return False
 
        return True        
        
if __name__ == "__main__":

    scenario = 1

    cfg = mno.Config(scenario)

    pallets = mno.loadPallets(cfg)

    # for p in pallets:
        # print(p.ID, p.D, p.W, p.V, p.Dests)

    # pallets capacity
    cfg.weiCap = 0
    cfg.volCap = 0
    for p in pallets:
        cfg.weiCap += p.W
        cfg.volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if cfg.weiCap > cfg.payload:
        cfg.weiCap = cfg.payload 

    # print(cfg.weiCap, cfg.volCap)

    inst = 1

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

    # print(len(items))

    mno.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)

    # for p in pallets:
        # print(p.ID, p.D, p.W, p.V, p.Dests)

    sol = Sol(pallets, items, cfg)

    limit = 1.0

    edges = mno.mountEdges(pallets, items, cfg)

    for ce in edges:

        if sol.isFeasible(ce, limit, cfg, k):
            sol.put(ce)

    print("---------")
    print(f"Sol. epsilon = {sol.T/cfg.maxTorque:.2f}\tSol. score = {sol.S}")

    print("---------")
    for p, vol in enumerate(sol.PCV):
        print(f"{p}\t{vol:.1f}\t{vol/pallets[p].V:.3f}")
    
    print("---------")
    for p, wei in enumerate(sol.PCW):
        print(f"{p}\t{wei:.0f}\t{wei/pallets[p].W:.2f}")        