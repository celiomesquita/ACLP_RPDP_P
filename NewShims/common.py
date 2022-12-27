
import os
import math
import numpy as np

CITIES = ["GRU", "GIG", "SSA", "CNF", "CWB", "BSB", "REC"]

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
        self.Attr = 0.0


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
        
        if ret:
            with lock:
                newTorque = abs(solTorque.value + deltaTau)
                if newTorque > abs(solTorque.value):
                    if newTorque > cfg.maxTorque:
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


def writeNodeCons(scenario, instance, cons, pi, node, surplus):

    dirname = f"./{surplus}/scenario_{scenario}/instance_{instance}"

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
def loadNodeCons(surplus, scenario, instance, pi, node, id):
    """
    Loads consolidated contents file for this instance, tour and node k
    """
    dirname = f"./{surplus}/scenario_{scenario}/instance_{instance}"
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
def loadNodeItems(scenario, instance, node, unatended, surplus): # unatended, future nodes
    """
    Load this node to unnatended items attributes
    """
    dirname = f"./{surplus}/scenario_{scenario}/instance_{instance}"
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
          
    items.sort(key=lambda x: abs(x.S/x.V), reverse=True)
    id = 0
    bestAttr = items[0].S / items[0].V # the first item has the best attractiveness
    for i, it in enumerate(items):
        items[i].Attr = (it.S/it.V) / bestAttr
        items[i].ID = id
        id += 1

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


def writeTourSol(method, scenario, instance, pi, tour, cfg, pallets, cons, write, surplus):

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

    # print(sol)

    if write:
        dirname = f"./results/{surplus}/{method}_{scenario}"
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

    print("----- Please execute module main -----")
