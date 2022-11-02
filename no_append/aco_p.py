import time
import math
import numpy as np
import methods as mno
from joblib import Parallel, delayed

# Process-based parallelism

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = 4   # number of ants for team

def rouletteSelection(values):
    sumVal = sum([v for v in values])
    probabilities = [v/sumVal for v    in           values ]
    indexes       = [i        for i, _ in enumerate(values)]
    return np.random.choice(indexes, p=probabilities)

# getDeltaTau calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar, numAnts):
    DeltaTau = score / bestSoFar # at this point DeltaTau is greater than 1.0
    DeltaTau /= numAnts*numAnts
    return DeltaTau

# each Ant makes use of "updatePheroAttract" to update pheromones
# and edge attractiveness according to the its solution value
def updatePheroAttract(score, bestSoFar, edges, numAnts, reset=False):

    # if True resets pheromone and attractiveness levels to diversify the search
    if reset:
        for i, _ in enumerate(edges):
            edges[i].Pheromone = 0.5
            edges[i].updateAttract(ALPHA, BETA)
        return      

    # evaporate some pheromone from all edges
    for id, e in enumerate(edges):
        # flatten pheromone curve
        edges[id].Pheromone = math.sqrt(e.Pheromone) / 1.25 # RHO = 0.2
        edges[id].updateAttract(ALPHA, BETA)

    deltaTau = getDeltaTau(score, bestSoFar, numAnts)

    if deltaTau < 1:
        # deposit some pheromone in the edges
        for id, e in enumerate(edges):
            if e.Pheromone < 1.0:
                edges[id].Pheromone += deltaTau
                edges[id].updateAttract(ALPHA, BETA)

# pick and delete an edge from the neighborhood by a proportional roulette wheel
def pickFromNbhood(nbhood, values):
    i = rouletteSelection(values)
    e = nbhood[i]
    nbhood.pop(i)
    values.pop(i)
    return e

def antSolve(Gant, cfg, k):
    Nbhood   = [ce for ce in Gant.Edges if not ce.InSol]
    attracts = [ce.Attract for ce in Nbhood]
    while Nbhood:
        ce = pickFromNbhood(Nbhood, attracts)
        if Gant.isFeasible(ce, 1.0, cfg, k):
            Gant.putInSol(ce)            
    return Gant

def antQueue( Gant, cfg, k, outQueue ):
    outQueue.put( antSolve(Gant, cfg, k) )

def Solve( pallets, items, startTime, cfg, k, limit):  # items include kept on board

    print("\nMultiprocess Ant Colony Optimization for ACLP+RPDP")

    antsField = mno.mountEdges(pallets, items, cfg)
    edges     = mno.mountEdges(pallets, items, cfg)

    # initialize the best solution so far           1.0 totally greedy
    Gbest = mno.Solution(antsField, pallets, items, 1.00, cfg, k)
    Ginit = mno.Solution(    edges, pallets, items, limit, cfg, k)

    numPallets = len(pallets)
    numItems   = len(items)
    # numAnts = 0
    stagnant = 0
   

    while stagnant < NANTS:# and (time.perf_counter() - startTime) < mno.SEC_BREAK:

        Glocal = Gbest

        sols = Parallel(n_jobs=NANTS)( delayed(antSolve)(Ginit, cfg, k)  for _ in range(NANTS) ) 

        for Gant in sols:
            if Gant.S > Glocal.S:
                Glocal = Gant

        if Glocal.S > Gbest.S:
            updatePheroAttract(Glocal.S, Gbest.S, antsField, NANTS)
            Gbest = Glocal
            stagnant = 0
        else:
            stagnant += 1

        if stagnant == 2: # reset pheromone level (True) to diversify the search
            updatePheroAttract(0, 0, antsField, 0, True)

    # print(f"Number of ants used: {numAnts}")

    return mno.getSolMatrix(Gbest.Edges, numPallets, numItems)

        
if __name__ == "__main__":


    mno.DATA = "data20" # 0.41 sce 1 | 0. sce 2
    # mno.DATA = "data50" # 0.55 sce 1 | 0. sce 2
    # mno.DATA = "data100"
  
    method = "ACO-mp"

    scenario = 1

    cfg = mno.Config(scenario)

    if scenario == 1:
        # instances = [1,2,3,4,5,6,7]
        instances = [1]
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

        startNodeTime = time.perf_counter()

        E = Solve( pallets, items, startNodeTime, cfg, k, 0.5)

        elapsed = time.perf_counter() - startNodeTime

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
                wNodeAccum += float(consJK[i][k].W)
                vNodeAccum += float(consJK[i][k].V)
                tau        += float(consJK[i][k].W) * pallets[i].D

            epsilom = tau/cfg.maxTorque

            sol += f"Score: {sNodeAccum}\t"
            sol += f"Weight: {wNodeAccum/cfg.weiCap:.2f}\t"
            sol += f"Volume: {vNodeAccum/cfg.volCap:.2f}\t"
            sol += f"Torque: {epsilom:.2f}\n"
            sol += f"Elapsed: {elapsed:.1f}s\n"

    print(sol)



