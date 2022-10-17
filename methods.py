
from os import path
import os
import math
import itertools
import numpy as np
from time import time

SEC_BREAK = 0.7

RNG = np.random.default_rng()

CITIES = ["GRU", "GIG", "SSA", "CNF", "CWB", "BSB", "REC"]

DATA = "data20"

class Node(object):
    def __init__(self, id, tau):
        self.ID      = id
        self.tau     = tau # node sum of torques
        self.ICAO    = CITIES[id]

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
        # self.Dests = [-1 for _ in range(numNodes)] # one destination for node
        self.Dests = np.full(shape=numNodes, fill_value=-1)

# Edge connecting a pallet and an item
class Edge(object):
    def __init__(self, id, pallet, item, cfg):
        self.ID        = id
        self.Pallet    = pallet
        self.Item      = item
        self.Torque    = float(item.W) * float(pallet.D)

        # 1500 to make the heuristic average around 1.0
        # devided by volume, because it is the most constraintive attribute
        # 5100 = 15m x 340kg, maximum torque possible
        factor = abs(self.Torque) / (cfg.maxD*340) # less than 1
        penalty = self.Item.S * factor
        self.Heuristic = float(item.S - penalty)**2 / (1500.0*float(item.V))

        self.Pheromone = 0.5# for ACO
        self.Attract   = self.Heuristic# for ACO
        self.Tested    = 0
        self.InSol     = False

    # for ACO
    def updateAttract(self, Alpha, Beta):
        self.Attract = self.Pheromone**Alpha + self.Heuristic**Beta

class Solution(object):
    def __init__(self, edges, pallets, items, limit, cfg, k):
       
        self.Edges = edges # set of edges

        self.S = 0 # solution total score
        self.W = 0 # solution total weight
        self.V = 0 # solution total volume

        # set of number of times items were included in solution
        self.Included = np.full(shape=len(items), fill_value=0)

        # pallets initial torque: 140kg times pallet CG distance
        self.T = sum(140 * p.D for p in pallets)  # solution total torque

        # self.PAW = [ 0   for _ in pallets] # set of pallets accumulated weights
        # self.PAV = [ 0.0 for _ in pallets] # set of pallets accumulated volumes

        self.PAW = np.full(shape=len(pallets), fill_value=0)
        self.PAV = np.full(shape=len(pallets), fill_value=0.0)

        for e in edges:           
            # insert consolidated in the solution
            # if defined by OptCGCons MIP procedure for optimal consolidated positions
            if e.Item.P == e.Pallet.ID:
                self.includeEdge(e) # does not ask if consolidated inclusion is feasible

        # builds a greedy solution until the limited capacity is attained
        if limit > 0:
            for ce in self.Edges:
                if not ce.InSol and self.isFeasible(ce, limit, cfg, k): # All constraints
                    self.includeEdge(ce)
                
    def includeEdge(self, e):
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
        if  abs(self.T) < abs(self.T + ce.Torque) and abs(self.T + ce.Torque) > cfg.aircraft.maxTorque:
            return False #this item/pallet torque would extend the CG shift beyond backward limit. Equation 15

        return True


def mountEdges(pallets, items, cfg, k):
    
    # items include kept on board, are from this node (k), and destined to unattended nodes
    m = len(pallets)
    n = len(items)

    edges = np.full(shape=m*n, fill_value=None)

    id = 0
    for p in pallets:           
        for it in items:
            e = Edge(id, p, it, cfg)
            edges[id] = e
            id += 1

    inds = np.array([e.Heuristic for e in edges])
    sort_inds = np.argsort(inds)
    edges = [edges[ind] for ind in sort_inds]

    return edges

# Proportional Roulette Selection (biased if greediness > 0)
def select(values, sumVal, greediness, sense):

    n = len(values)
    if n == 0:
        return -1
    if n == 1:
        return 0

    threshold = RNG.random()*(1.0-greediness) + greediness
    threshold *= sumVal

    maxVal = max(values)

    pointer = 0.0
    for j, v in enumerate(values):

        if sense == -1: # preference for worst values
            pointer += maxVal - v
        else:
            pointer += v

        if pointer >= threshold: # when pointer reaches the threshold
            return j # returns the chosen index

    # just in case ....
    return int(n * RNG.random())

def loadDistances():
    fname =  f"distances.txt"      
    with open(fname, 'r') as f:
        distances = [ [ float(num) for num in line.split(' ') ] for line in f ] 
    return distances 

# tour is pi in the mathematical formulation
class Tour(object):
    def __init__(self, nodes, costs):
        self.nodes = nodes
        self.cost  = 0.0 # sum of legs costs plus CG deviation costs
        self.legsCosts = [0 for _ in nodes]
        self.score   = -1 # sum of nodes scores
        self.elapsed = -1 # seconds
        self.numOpts = 0 # sum of nodes eventual optima solutions
        self.bestSC  = 0 # best ratio score/cost of this tour

        for k, node in enumerate(self.nodes):

            if k > 0:
                frm = self.nodes[k-1].ID
                to  = node.ID                                
                self.legsCosts[k] = costs[frm][to]
                self.cost += self.legsCosts[k]

def getTours(numNodes, costs):

    last = numNodes - 1
    ids = [i+1 for i in range(last)]
    permuts = itertools.permutations(ids)

    tours = []

    for p in permuts:

        tau = 0 
        nodes = []
        nodes.append( Node(0, tau) ) # the base
        for i in p:
            nodes.append( Node(i, tau) )
        nodes.append( Node(0, tau) ) # the base

        tours.append( Tour(nodes, costs) )

    return tours

class Aircraft(object):
    def __init__(self, size, numPallets, payload, maxShift, kmCost):
        self.size       = size
        self.numPallets = numPallets
        self.payload    = payload
        self.maxTorque  = payload * maxShift
        self.kmCost     = kmCost

class Config(object):
    """
    Problem configuration
    """
    def __init__(self, scenario):

        self.weiCap = 0
        self.volCap = 0
        self.maxD   = 0

        self.numNodes = {0:3,   1:3,   2:3,   3:4,   4:5,   5:6,   6:7}[scenario]
        self.Sce      = {0:1,   1:1,   2:2,   3:3,   4:4,   5:5,   6:6}[scenario]

        self.aircraft = {
            0:Aircraft("smaller", 7, 26_000, 0.556, 1.1), # 3 nodes,  7 pallets, no time limit
            1:Aircraft("smaller", 7, 26_000, 0.556, 1.1), # 3 nodes,  7 pallets, 600s
            2:Aircraft("larger", 18, 75_000, 1.170, 4.9), # 3 nodes, 18 pallets, 600s
            3:Aircraft("larger", 18, 75_000, 1.170, 4.9), # 4 nodes, 18 pallets, 600s
            4:Aircraft("larger", 18, 75_000, 1.170, 4.9), # 5 nodes, 18 pallets, 600s
            5:Aircraft("larger", 18, 75_000, 1.170, 4.9), # 6 nodes, 18 pallets, 600s
            6:Aircraft("larger", 18, 75_000, 1.170, 4.9)  # 7 nodes, 18 pallets, 600s
            }[scenario]


def loadPallets(cfg):
    """
    Load pallets attributes based on aircraft size
    """
    fname = f"{cfg.aircraft.size}.txt"
      
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

    two_up = path.abspath(path.join(__file__, "../..")) + f"/{DATA}"
    dirname = f"/{two_up}/scenario_{scenario}/instance_{instance}"

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
    two_up = path.abspath(path.join(__file__, "../..")) + f"/{DATA}"
    dirname = f"/{two_up}/scenario_{scenario}/instance_{instance}"
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

# used in systemic mode, the base case
def loadItems(scenario, instance, id, type): # -1: items, -2: consolidated
    """
    Load this node items attributes
    """
    two_up = path.abspath(path.join(__file__, "../..")) + f"/{DATA}"

    dirname = f"/{two_up}/scenario_{scenario}/instance_{instance}"

    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass    

    fname = "%s/items.txt" % (dirname)

    if not os.path.exists(fname):
        with open(fname, "r"):
            pass

    reader = open(fname)
    lines = reader.readlines() 

    items = []

    try:
        for line in lines:
            cols = line.split()
            w   =   int(cols[0])
            s   =   int(cols[1])
            v   = float(cols[2])
            frm = int(cols[3])
            to  =   int(cols[4])
            items.append( Item(id, type, w, s, v, frm, to) ) # type: -1 items -2 consolidated, >=0 pallet position
            id += 1

    finally:
        reader.close()    

    return items

# used in sequential mode
def loadNodeItems(scenario, instance, node, unatended): # unatended,future nodes
    """
    Load this node items attributes
    """
    two_up = path.abspath(path.join(__file__, "../..")) + f"/{DATA}"

    dirname = f"/{two_up}/scenario_{scenario}/instance_{instance}"
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

    # vol       = np.full(shape=len(nodes), fill_value=0)
    # PalConsol = np.full(shape=len(nodes), fill_value=0)

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

def loadFloats(fname):

    reader = open(fname,"r")
    vector = []
    try:
        line = reader.readline() # read line by line
        while line != '':
            vector.append(float(line))
            line = reader.readline()
    finally:
        reader.close()
    return(vector)
  
def writeBestInFile(fname, value, elapsed, opt):
    """
    If value is greater than first column in file, update the file
    """
    reader = open(fname, "r+")
    best = 0
    try:
        line = reader.readline()
        if line != '':
            cols = line.split()
            best = int(cols[0])
    finally:
        reader.close()   

    if value > best:
        writer = open(fname, "w") 
        try:
            writer.write("%d %d %d" % (value, elapsed, opt))
        finally:
            writer.close()

def writeResult(fname, value):

    writer = open(fname, "w+") 
    try:
        writer.write(value)
    finally:
        writer.close()        



def writeTourSol(method, scenario, instance, pi, tour, cfg, pallets, cons, write):

    '''this is the commom solution print method for all solvers'''

    cfg.volCap = 0.
    palletsWei = 0.
    for i, p in enumerate(pallets):
        cfg.volCap += p.V
        palletsWei += p.W

    cfg.weiCap = float(min(cfg.aircraft.payload, palletsWei))

    sTourAccum = 0

    if write:
        sol = '\\begin{tabular}{c c c c} \n'

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

        epsilom = tau/cfg.aircraft.maxTorque

        if write:
            sol += '\midrule \n'
            sol += f"Score: {sNodeAccum} & "
            sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f} & "
            sol += f"Volume: {vNodeAccum/cfg.volCap:.2f} & "
            sol += f"Torque: {epsilom:.2f} \\\\ \n"

        sTourAccum += sNodeAccum

        # Equation 3: 5% penalty at most, due to CG unbalance
        tour.cost += abs(epsilom) * (tour.legsCosts[k+1] / 20)

    if write:
        sol += '\\bottomrule \n'
        sol += '\multicolumn{4}{l}{' + f"Tour {pi}\ \ \ \ \ S/C: {float(sTourAccum)/tour.cost:.3f}\ \ \ \ \ "


    tour.score = float(sTourAccum)

    if write:
            
        timeString = getTimeString(tour.elapsed, 1) # 1 instance            

        sol += f"\ \ \ \ \ elapsed: {timeString}\ \ \ \ \ method:\ {method}" + '} \\\\ \n'

        sol += '\\bottomrule \n'
        sol += '\end{tabular} \n'

        dirname = f"./results/{method}_{scenario}"
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
  


# for latex table --------------
class Cell(object):
    """
    Cell solution
    """
    def __init__(self, scenario, instance, pi, node, time, score, weight, opt):
        self.scenario = scenario
        self.instance = instance
        self.pi      = pi
        self.node    = node
        self.time     = time
        self.score    = score
        self.weight   = weight
        self.opt      = opt
     

def getLine(n, cell, maxi, len_inst):

    line = ""

    celltime = "%d" % (cell.time)
    if cell.time > 60000:
        celltime = "\\bf{%s}" % (celltime)

    if maxi == "s":
        if cell.instance == 1:
            if n == 0:
                line += " %d & %d & %s & %d & %d &" % (cell.scenario, cell.node, celltime, cell.score, cell.opt)
            else:
                line += "    & %d & %s & %d & %d &" % (               cell.node, celltime, cell.score, cell.opt)
        
        if len_inst == 1:
            line += "\n"

        if cell.instance > 1 and cell.instance < 5:
            line += " %s & %d & %d &" % (celltime, cell.score, cell.opt)

        if len_inst > 1 and len_inst < 5:
            line += "\n"

        if cell.instance == 5:
            line += " %s & %d & %d " % (celltime, cell.score, cell.opt)
            line += "\\\\\n"

    if maxi == "w":
        if cell.instance == 1:
            if n == 0:
                line += " %d & %d & %s & %d & %d &" % (cell.scenario, cell.node, celltime, cell.weight, cell.opt)
            else:
                line += "    & %d & %s & %d & %d &" % (               cell.node, celltime, cell.weight, cell.opt)
        
            if len_inst == 1:
                line += "\n"  
                  
        if cell.instance > 1 and cell.instance < 5:
            line += " %s & %d & %d &" % (celltime, cell.weight, cell.opt)

            if len_inst > 1 and len_inst < 5:
                line += "\n"

        if cell.instance == 5:
            line += " %s & %d & %d " % (celltime, cell.weight, cell.opt)
            line += "\\\\\n"

    return line


def getLine2(n, cell, len_inst):

    line = ""

    if cell.instance == 1:
        line += "%d %d %d " % (cell.scenario, cell.node, cell.time)
    
    if len_inst == 1:
        line += "\n"

    if cell.instance > 1 and cell.instance < 5:
        line += "%d " % (cell.time)

    if len_inst > 1 and len_inst < 5:
        line += "\n"

    if cell.instance == 5:
        line += "%d" % (cell.time)
        line += "\n"

    return line 

def getTimeString(totTime, denom):

    totTime = totTime / denom
    totTimeS = f"{totTime:.2f}s"

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

if __name__ == "__main__":

    scenario = 6

    instances = 1

    dists = loadDistances()

    cfg = Config(scenario)

    tours = getTours(cfg.numNodes, dists)

    for pi, tour in enumerate(tours):

        print(f"{pi}: ", end='')
        for node in tour.nodes:
            print(f"{node.ICAO} ", end='')
        print(f" {tour.cost}")


'''
      GRU     GIG     SSA     CNF     CWB     BSB     REC
GRU     0	 6741	28118	 9477	 6898	16922	41289
GIG  6741	    0	23683	 6644	13190	18016	36462
SSA 28118	23683	    0	18895	46428	20732	13170
CNF  9477	 6644	18895	    0	16179	12017	31890
CWB  6898	13190	46428	16179	    0	21123	48089
BSB 16922	18016	20732	12017	21123	    0	32398
REC 41289	36462	13170	31890	48089	32398	    0
'''


'''
    # FOR TESTING ONLY
    # DO NOT CHANGE THESE PARAMETERS
    scenario = 4
    instance = 1
    pi      = 0  # tour index in the permutations
    tour     = [0, 1, 2, 3, 4, 0]

    # this node ID may be 0 or 1 for testing    
    node = Node(0, 0.0)

    cfg = Config(scenario)

    pallets = loadPallets(cfg)

    print("len(pallets)", len(pallets))

    k = tour.index(node)  # node index in the tour
    unattended = []
    for node in tour[k+1:]:
        unattended.append(node.ID)

    print("len(unattended)", len(unattended))

    items = loadNodeItems(scenario, instance, node, unattended)

    print("len(items)", len(items))

    # pallets capacity
    weiCap = 0
    volCap = 0
    for p in pallets:
        weiCap += p.W
        volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if weiCap > cfg.aircraft.payload:
        weiCap = cfg.aircraft.payload


# 1) Iterate with enumerate instead or np.arange(len(x))
# 2) Use list comprehension instead of raw for loops
# 3) Sort complex iterables with sorted()
# 4) Store unique values with Sets
# 5) Save memory with Generators
# 6) Define default values in Dictionaries with .get() and .setdefault()
# 7) Count hashable objects with collections.Counter
# 8) Format strings with f-Strings (Python 3.6+)
# 9) Concatenate strings with .join()
# 10) Merge dictionaries with {**d1, **d2} (Python 3.5+)
# 11) Simplify if-statements with if x in list


# Roullete selection testing
    # values = [
    #     0.0136, 0.0439, 0.1051, 0.2356, 0.2940, 0.3122, 0.3658,
    #     0.4455, 0.4459, 0.4650, 0.4976, 0.6192, 0.6497, 0.7206,
    #     0.7377, 0.8407, 0.8659, 0.9898, 1.1237, 1.2347, 1.3457 ]
'''

'''
    values = [None]*len(items)
    for i, it in enumerate(items):
        values[i] = float(it.S)**2 / 1500*it.V

    sumVal = sum(values)
    maxVal = max(values)

    counts = [0]*len(values)
    startTime = time()

    for _ in range(len(items)):
        i = randomProportionalSelection(values, maxVal)
        counts[i] += 1

    end = time() - startTime

    print(f"\nProportional elapsed: {math.ceil(1000*end)}")
    for c in counts:
        if c > 0:
            print(f"{c} ", end='')
    print()

    var = np.var(counts)
    print(f"Proportional counts variance: {var:.2f}")

    counts = [0]*len(values)
    startTime = time()

    for _ in range(len(items)):
        i = select(values, sumVal, 0.1)
        counts[i] += 1

    end = time() - startTime

    print(f"\nRoulette elapsed: {math.ceil(1000*end)}")
    for c in counts:
        if c > 0:
            print(f"{c} ", end='')
    print()

    var = np.var(counts)
    print(f"Roulette counts variance: {var:.2f}")
'''
