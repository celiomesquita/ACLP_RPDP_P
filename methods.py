
from os import path
import os
import math
import numpy as np
from time import time
import multiprocessing as mp

CITIES = ["GRU", "GIG", "SSA", "CNF", "CWB", "BSB", "REC"]

DATA = "data20"

class Node(object):
    def __init__(self, id, tau):
        self.ID      = id
        self.tau     = tau # node sum of torques
        self.ICAO = CITIES[0]
        if id < len(CITIES):
            self.ICAO = CITIES[id]

class Item(object):
    """
    A candidate item "j" to be loaded on a pallet "i" if X_ij == 1
    """
    def __init__(self, id, p, w, s, v, frm, to):
        self.ID = id
        self.P  = p  # -1 if an item, -2 if a consollidated, or pallet ID.
        self.W  = w  # weight
        self.S  = s  # score
        self.V  = v  # volume
        self.Frm = frm  # from
        self.To = to # destination

class Pallet(object):
    """
    A flat metal surface to hold items to be transported in a airplane
    """
    def __init__(self, id, d, v, w, numNodes):
        self.ID = id
        self.D  = d  # centroid distance to CG
        self.V  = v  # volume limit
        self.W  = w  # weight limit
        self.Dests = np.full(numNodes, -1)

# Edge connecting a pallet and an item
class Edge(object):
    def __init__(self, id, pallet, item, cfg, Alpha, Beta):

        self.ID      = id # index in the solution edges (it must be preserved in case of sorting)
        self.Pallet  = pallet
        self.Item    = item
        self.Torque  = float(item.W) * float(pallet.D)

        # 1500 to make the heuristic average around 1.0
        # devided by volume, because it is the most constraintive attribute
        # 5100 = 15m x 340kg, maximum torque possible
        factor = abs(self.Torque) / (cfg.maxD*340) # less than 1

        self.Heuristic = ( float(item.S) / ( 3000 * float(item.V) ) ) * (1.02 - factor)

        self.Pheromone = 0.5# for ACO
        self.Attract   = self.Pheromone**Alpha + self.Heuristic**Beta # for ACO
        self.InSol     = False

    # for ACO
    def updateAttract(self, Alpha, Beta):
        self.Attract = self.Pheromone**Alpha + self.Heuristic**Beta

def edgesCopy(edges):
    output = edges.copy()
    for i, e in enumerate(edges):
        output[i].ID        = e.ID
        output[i].Pallet    = e.Pallet 
        output[i].Item      = e.Item 
        output[i].Torque    = e.Torque 
        output[i].Heuristic = e.Heuristic 
        output[i].Pheromone = e.Pheromone 
        output[i].Attract   = e.Attract
        output[i].InSol     = False # reset solution
    return output
  
class Solution(object):
    def __init__(self, edges, pallets, items, limit, cfg, k):

        self.Limit = limit     

        self.Edges = edgesCopy(edges)

        self.S = 0 # solution total score
        self.W = 0 # solution total weight
        self.V = 0 # solution total volume
        self.Heuristic = 0 

        # set of number of times items were included in solution
        self.Included = [ 0  for _ in items ] 

        # pallets initial torque: 140kg times pallet CG distance
        self.T = sum(140 * p.D for p in pallets)  # solution total torque

        self.PAW = [ 0   for _ in pallets] # set of pallets accumulated weights
        self.PAV = [ 0.0 for _ in pallets] # set of pallets accumulated volumes

        # builds a greedy solution until the limited capacity is attained
        if limit > 0:
            for ce in self.Edges:
                if not ce.InSol and self.isFeasible(ce, limit, cfg, k):
                    self.putInSol(ce)

    def putInSol(self, e):
        if e.Item.ID > len(self.Included)-1:
            return
        if e.Pallet.ID > len(self.PAW)-1:
            return
        self.Included[e.Item.ID] += 1
        self.PAW[e.Pallet.ID] += e.Item.W
        self.PAV[e.Pallet.ID] += e.Item.V
        self.S += e.Item.S
        self.W += e.Item.W
        self.V += e.Item.V
        self.T += e.Torque
        self.Heuristic += e.Heuristic  
        self.Edges[e.ID].InSol = True

    # check constraints for greedy, Shims and metaheuristics
    def isFeasible(self, ce, limit, cfg, k):

        if ce.Item.ID > len(self.Included)-1:
            return False # this item was already inserted. Equation 19        

        if self.Included[ce.Item.ID] > 0:
            return False # this item was already inserted. Equation 19
        
        if ce.Item.To != ce.Pallet.Dests[k]:
            return False #item and pallet destinations are different. Equation 21
        
        # Pallet Acumulated Weight: cancelled because the most constraintive is volume
        if self.PAW[ce.Pallet.ID] + ce.Item.W > ce.Pallet.W:
             return False #this item weight would exceed pallet weight limit. Equation 17
        
        # Pallet Acumulated Volume
        if self.PAV[ce.Pallet.ID] + ce.Item.V > ce.Pallet.V * limit:
            return False #this item volume would exceed pallet volumetric limit. Equation 18
        
        # if this inclusion increases torque and is turns it greater than the maximum
        newTorque = abs(self.T + ce.Torque)
        if newTorque > cfg.maxTorque:
            return False
 
        return True

def getLimits(minLim, numProcs): # pid: 0 - numProcs-1

    limitSet = set()

    maxLim = 0.98

    delta = (maxLim - minLim)/(numProcs+1)

    # set of unique limit values
    for i in range(numProcs):
        limitSet.add(minLim + (i+1)*delta)

    # return list(limitSet)
    return [*limitSet, ] # a bit faster
    
# mount the decision matrix for which items will be put in which pallets
def getSolMatrix(edges, numPallets, numItems):
    X = np.zeros((numPallets,numItems))
    for e in edges:
        if e.InSol:
            X[e.Pallet.ID][e.Item.ID] = 1
    return X

def loadDistances():
    fname =  f"./params/distances.txt"      
    with open(fname, 'r') as f:
        distances = [ [ float(num) for num in line.split(' ') ] for line in f ] 
    return distances 

# tour is pi in the mathematical formulation
class Tour(object):
    def __init__(self, nodes, cost):
        self.nodes = nodes
        self.cost  = cost # sum of legs costs plus CG deviation costs
        self.score   = -1 # sum of nodes scores
        self.elapsed = -1 # seconds
        self.numOpts = 0 # sum of nodes eventual optima solutions
        self.bestSC  = 0 # best ratio score/cost of this tour

class Config(object):

    def __init__(self, scenario):
        self.weiCap = 0
        self.volCap = 0
        self.maxD   = 0
        self.numNodes = {0:3,  1:3,  2:3,    3:4,    4:5,    5:6,    6:7    }[scenario]
        self.Sce      = {0:1,  1:1,  2:2,    3:3,    4:4,    5:5,    6:6    }[scenario]

        self.size       = "smaller"
        self.numPallets = 7
        self.payload    = 26_000
        self.maxTorque  = 26_000 * 0.556
        self.kmCost     = 1.1
        if scenario > 1:
            self.size       = "larger"
            self.numPallets = 18
            self.payload    = 75_000
            self.maxTorque  = 75_000 * 1.170
            self.kmCost     = 4.9 

def factorial(x):
    result = 1
    for i in range(x):
        result *= i+1
    return result

def permutations(n):
    fac = factorial(n)
    a = np.zeros((fac, n), np.uint32) # no jit
    f = 1
    for m in np.arange(2, n+1):
        b = a[:f, n-m+1:]      # the block of permutations of range(m-1)
        for i in np.arange(1, m):
            a[i*f:(i+1)*f, n-m] = i
            a[i*f:(i+1)*f, n-m+1:] = b + (b >= i)
        b += 1
        f *= m
    return a

def getTours(num, costs, threshold):

    p = permutations(num)

    #+2: the base before and after the permutation
    toursInt = [[0 for _ in np.arange(len(p[0])+2)] for _ in np.arange(len(p))]

    # define the core of the tours
    for i, row in enumerate(p):
        for j, col in enumerate(row):
            toursInt[i][j+1] = col+1

    tours = [None for _ in np.arange(len(toursInt))]
    minCost = 9999999999999.
    maxCost = 0.      
    for i, tour in enumerate(toursInt):
        nodes = []
        cost = 0.
      
        for j, nid in enumerate(tour[:-1]):
            n = Node(nid, 0.)
            nodes.append( n )

            if j>0:
                frm = nodes[j-1].ID
                to  = nodes[j].ID
                if frm < len(costs) and to < len(costs[frm]):
                    cost += costs[frm][to]

            if j == len(tour[:-1])-1: # the last node
                frm = nodes[j].ID
                to  = 0
                if frm < len(costs) and to < len(costs[frm]):
                    cost += costs[frm][to]                

        if cost < minCost:
            minCost = cost

        if cost > maxCost:
            maxCost = cost

        tours[i] = Tour(nodes, cost)

    tours2 = []
    for i, t in enumerate(tours):
        if t.cost <= minCost + (maxCost - minCost) * threshold:
            tours2.append(t)
    tours    = None
    toursInt = None

    return tours2

def mountEdges(pallets, items, cfg, Alpha=1, Beta=4):
   
    # items include kept on board, are from this node (k), and destined to unattended nodes
    m = len(pallets)
    n = len(items)

    edges = [None for _ in np.arange(m*n)]

    mpAttract = mp.Array('d', [0.5 for _ in np.arange(m*n)])

    i = 0
    for p in pallets:           
        for it in items:
            e = Edge(i, p, it, cfg, Alpha, Beta)
            edges[i] = e
            i += 1

    # greater heuristic's attractivity are included first
    edges.sort(key=lambda x: x.Heuristic, reverse=True) # best result

    # as edges have its order changed the ID must in the original order
    # to preserve reference
    for i, e in enumerate(edges):
        edges[i].ID = i

    return edges, mpAttract
    
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

def writeNodeCons(scenario, instance, cons, pi, node):

    dirname = f"./{DATA}/scenario_{scenario}/instance_{instance}"

    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass    

    fname = f"{dirname}/cons_{pi}_{node.ID}.txt" # pi is a tour index

    lines = ""
    for c in cons:
        line = f"{c.W} {c.S} {c.V:.3f} {c.Frm} {c.To}\n"
        lines += line

    writer = open(fname, "w+") 
    try:
        writer.write(lines)
    finally:
        writer.close()  

# used in sequential mode
def loadNodeCons(scenario, instance, pi, node, id):
    """
    Loads consolidated contents file for this instance, tour and node k
    """
    dirname = f"./{DATA}/scenario_{scenario}/instance_{instance}"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass    

    fname = f"{dirname}/cons_{pi}_{node.ID}.txt"

    reader = open(fname,"r")
    lines = reader.readlines() 

    cons = []
    try:
        for line in lines:
            cols = line.split()
            w   =   int(cols[0])
            s   =   int(cols[1])
            v   = float(cols[2])
            frm =   int(cols[3])
            to  =   int(cols[4])         
            if w > 0: #           P = -2 consolidated
                cons.append( Item(id, -2, w, s, v, frm, to) )
                id += 1
    finally:
        reader.close()

    return cons

# used in sequential mode
def loadNodeItems(scenario, instance, node, unatended): # unatended, future nodes
    """
    Load this node to unnatended items attributes
    """
    dirname = f"./{DATA}/scenario_{scenario}/instance_{instance}"
    fname = f"{dirname}/items.txt"

    reader = open(fname, "r")
    lines = reader.readlines() 

    items = []
    id = 0
    try:
        for line in lines:
            cols = line.split()
            w   =   int(cols[0])
            s   =   int(cols[1])
            v   = float(cols[2])
            frm =   int(cols[3])
            to  =   int(cols[4])
            if frm == node.ID and to in unatended:          
                items.append( Item(id, -1, w, s, v, frm, to) ) # P:-1 item, -2: consolidated
                id += 1
    finally:
        reader.close()    

    return items


def setPalletsDestinations(items, pallets, nodes, k, L_k):

    vol       = [0]*len(nodes)
    PalConsol = [0]*len(nodes)
    max   = 0
    total = 0

    # all items from all nodes
    for it in items:
        # the items from this node
        if it.Frm == nodes[k].ID and it.P == -1:
            d = it.To
            if d in L_k:
                vol[d] += it.V
                total  += it.V
                if vol[d] > max:
                    max = d
    # all items from all nodes
    for it in items:
        # the consolidated from this node
        if it.Frm == nodes[k].ID and it.P == -2:    
            d = it.To
            PalConsol[d] += 1
            if d in L_k:
                vol[d] += it.V
                total  += it.V
                if vol[d] > max:
                    max = d
        
    for n in nodes:
        if vol[n.ID] > 0:
            np = math.floor( len(pallets) * vol[n.ID] / total)
            # quant = max(1, np - PalConsol[n.ID])
            quant = np - PalConsol[n.ID]
            count = 0
            for p in pallets:
                if count == quant:
                    break
                if p.Dests[k] == -1:
                    pallets[p.ID].Dests[k] = n.ID
                    count += 1

    for p in pallets:
        if p.Dests[k] == -1:
            pallets[p.ID].Dests[k] = max


 
def writeResult(fname, value):

    writer = open(fname, "w+") 
    try:
        writer.write(value)
    finally:
        writer.close()        


def getTimeString(totTime, denom, inSecs=False):

    totTime = totTime / denom
    totTimeS = f"{totTime:.0f}"

    if inSecs:
        return totTimeS

    if totTime > 60:
        totTime /= 60
        int_part  = math.floor(totTime)
        frac_part = totTime - int_part
        totTimeS = f"{int_part}min {frac_part*60:.0f}s"

        if totTime > 60.0:
            totTime /= 60
            int_part  = math.floor(totTime)
            frac_part = totTime - int_part
            totTimeS = f"{int_part}h {frac_part*60:.0f}min"

    return totTimeS


def writeTourSol(method, scenario, instance, pi, tour, cfg, pallets, cons, write):

    '''this is the commom solution print method for all solvers'''

    cfg.volCap = 0.
    palletsWei = 0.
    for i, p in enumerate(pallets):
        cfg.volCap += p.V
        palletsWei += p.W

    cfg.weiCap = float(min(cfg.payload, palletsWei))

    sTourAccum = 0

    if write:
        sol = '\\begin{tabular}{c c c c} \n'
    else:
        sol = "\n"

    for k, node in enumerate(tour.nodes[:cfg.numNodes]):

        sNodeAccum = 0.
        wNodeAccum = 0.
        vNodeAccum = 0.
        tau = 0.

        if write:
            sol += '\\toprule \n'
            sol += '\\rowcolor{Gray}'
            sol += '\multicolumn{4}{l}{' + f"Node {node.ID}: {CITIES[node.ID]} solution" + '} \\\\ \n'
            sol += '$Pallet$ & $From:To$ & $Kg$ & $m^3$ \\\\ \n'
            sol += '\midrule \n'

        for i, p in enumerate(pallets):

            if write:
                sol += f"{i+1} & {CITIES[node.ID]}:{CITIES[cons[i][k].To]} & "

            wNodeAccum += float(cons[i][k].W)
            vNodeAccum += float(cons[i][k].V)
            sNodeAccum += float(cons[i][k].S)
            tau += float(cons[i][k].W) * pallets[i].D

            wspace = ""
            if cons[i][k].W < 1000:
                wspace += " "
                if cons[i][k].W < 100:
                    wspace += " "
                    if cons[i][k].W < 10:
                        wspace += " "

            vspace = ""
            if cons[i][k].V < 10:
                vspace = " "

            if write:
                sol += f"{wspace}{cons[i][k].W} & {vspace}{cons[i][k].V:.1f} \\\\ \n"

        epsilom = tau/cfg.maxTorque
        tour.cost *= ( 1.0 + abs(epsilom)/20.0 )

        if write:
            sol += '\midrule \n'
            sol += f"Score: {sNodeAccum} & "
            sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f} & "
            sol += f"Volume: {vNodeAccum/cfg.volCap:.2f} & "
            sol += f"Torque: {epsilom:.2f} \\\\ \n"
        else:
            sol += f"Score: {sNodeAccum}\t"
            sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
            sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
            sol += f"Torque: {epsilom:.2f}\n"            

        sTourAccum += sNodeAccum

    if write:

        sol += '\\bottomrule \n'
        sol += '\multicolumn{4}{l}{' + f"Tour {pi}\ \ \ \ \ S/C: {float(sTourAccum)/tour.cost:.3f}\ \ \ \ \ "

    tour.score = float(sTourAccum)
            
    if write:                 
        timeString = getTimeString(tour.elapsed, 1, inSecs=True) # 1 instance   
        sol += f"\ \ \ \ \ elapsed: {timeString}\ \ \ \ \ method:\ {method}" + '} \\\\ \n'
        sol += '\\bottomrule \n'
        sol += '\end{tabular} \n'

    print(sol)

    if write:
        dirname = f"./results/{DATA}/{method}_{scenario}"
        try:
            os.makedirs(dirname)
        except FileExistsError:
            pass    

        fname = f"{dirname}/res_{method}_{scenario}_{instance}_{pi}.txt"
        writer = open(fname, "w+") 
        try:
            writer.write(sol)
        finally:
            writer.close() 

if __name__ == "__main__":

    scenario = 3

    cfg = Config(scenario)

    pallets = loadPallets(cfg)

    dists = loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value

    tours = getTours(cfg.numNodes-1, costs, 0.25)


    pi = 0 # the first, not necessarily the best

    tour = tours[pi]

    k = 0 # the base

    # L_k destination nodes set
    unattended = [n.ID for n in tour.nodes[k+1:]]

    node = tour.nodes[k]

    scenario = 1

    instance = 1

    items = loadNodeItems(scenario, instance, node, unattended)

    edges = mountEdges(pallets, items, cfg, 1, 3)

    maxHeu = max(e.Heuristic for e in edges)

    minHeu = min(e.Heuristic for e in edges)

    sumHeu = sum(e.Heuristic for e in edges)

    avgHeu = sumHeu/len(edges)

    print(minHeu, avgHeu, maxHeu)


    maxAttract = max(e.Attract for e in edges)

    minAttract = min(e.Attract for e in edges)

    sumAttract = sum(e.Attract for e in edges)

    avgAttract = sumAttract/len(edges)

    print(minAttract, avgAttract, maxAttract)


