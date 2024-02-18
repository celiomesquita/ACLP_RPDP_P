import time
import multiprocessing as mp
import random
import math
import numpy as np
import statistics
import copy
import common

RNG = np.random.default_rng()

# from tabulate import tabulate
# table =[]
# row = []
# row.append(col1)
# row.append(col2)
# table.append(row)
# print( tabulate(table, headers=['col1','col2']) )

ALPHA = 1 # pheromone exponent
BETA  = 4 # heuristic exponent

# Ant System (AS), the classic method that uses a random proportional state transition rule,
# while the pheromone is deposited by all ants proportionally to their solution quality and
# is evaporated in all the components. Each Ant makes use of "depositPhero" to update 
# item attractiveness according to the its solution value.
def depositPhero(antScore, iterScore, Attract, Phero, Edges):

    deltaTau = (antScore - iterScore)/iterScore

    # if deltaTau > 0: # if improved...

    for x, _ in enumerate(Phero):

        Phero[x] += deltaTau

        Attract[x] = Phero[x]**ALPHA * Edges[x].Attract**BETA

def evaporate(Attract, Phero, Edges):

        for x, phero in enumerate(Phero):

            #evaporate some pheromone 
            Phero[x] = math.sqrt(phero) / 1.35

            Attract[x] = Phero[x]**ALPHA * Edges[x].Attract**BETA

# at least 15 times faster than randomChoice
def rouletteSelection(attracts): 
    pick = RNG.random()*sum(attracts) # stop the roulette in a random point
    current = 0.
    for x, attr in enumerate(attracts): # walk on the roulette
        current += attr
        if current > pick:
            return x, attr # return the edge index
    return 0, 0

# for tournament selection
class Value(object):
    def __init__(self):
        self.x    = -1
        self.Attr = 0.0

# pick and delete an item from the neighborhood by a tournament selection
def pickAnEdge(nbhood, attracts):
      
    # make the tournament selection with 3 individuals
    individuals = [Value() for _ in range(3)]

    for i, _ in enumerate(individuals):
        individuals[i].x, individuals[i].Attr = rouletteSelection(attracts)

    # choose the best individual
    individuals.sort(key=lambda x: x.Attr, reverse=True)

    x = individuals[0].x

    # x, _ = rouletteSelection(attracts)

    edge = nbhood[x]

    nbhood.pop(x)
    attracts.pop(x)

    return edge

def antSolve(antPallets, edges, cfg, k, secBreak, antTorque, antSolDict, startTime, antScores, itemsDict, lock, n):

    nbhood   = [e for e in edges if itemsDict["mpItems"][e.Item.ID] == 0]
    attracts = [e.Phero**ALPHA * e.Attract**BETA for e in nbhood]

    while nbhood and (time.perf_counter() - startTime) < secBreak:
            
        if len(nbhood) > 0:

            # pick from the neighborhood the probable best item-pallet association
            edge = pickAnEdge(nbhood, attracts)         

            if antPallets[edge.Pallet.ID].isFeasible(edge.Item, 1.0, k, antTorque, cfg, itemsDict, lock):

                antPallets[edge.Pallet.ID].putItem(edge.Item, antTorque, antSolDict, n, itemsDict, lock)

                with lock:
                    antScores.value += edge.Item.S    


def Solve( pallets, items, cfg, pi, k, secBreak, mode, nodeTorque, solDict, itemsDict ):

    startTime = time.perf_counter()

    n = len(items)
    m = len(pallets)

    edges = [None]*n*m
    id = 0
    for p in pallets:
        for it in items:
            edges[id] = common.Edge(id, p, it)
            id += 1

    # to control the general attractiveness for the tournament selection
    Attract = mp.Array('d', np.arange(n*m) )
    for x, _ in enumerate(Attract):       
        Attract[x] = edges[x].Attract

    # to control pheromone deposition and evaporation
    Phero = mp.Array('d', np.arange(n*m) )
    for x, _ in enumerate(Phero):
        Phero[x] = 0.5


    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    print(f"\n{mode} Ant Colony Optimization for ACLP+RPDP (tour {pi} - node {k})")        
    print(f"{len(items)} items  {len(pallets)} pallets")

    lock  = mp.Lock()

    initPallets   = common.copyPallets(pallets)
    initSolDict   = dict(solDict)
    initItemsDict = dict(itemsDict)
    initTorque    = mp.Value('d', nodeTorque.value) 
    initScore     = mp.Value('d', 0)  

    for i, _ in enumerate(initPallets):               
        common.fillPallet(initPallets[i], items, k, initTorque, initSolDict, cfg, 1.0, initItemsDict, lock)
        initScore.value  += initPallets[i].PCS    

    iterScore     = mp.Value('d', initScore.value)
    bestAntScore  = mp.Value('d', initScore.value)

    iter         = 0
    stagnant     = 0
    numAnts      = mp.cpu_count() # number of ants per iteration
    improvements = 0


    while stagnant < 3: # iterations      

        # initialize ants parameters
        ants       = [a             for a in np.arange(numAnts)]
        antScores  = [mp.Value('d') for _ in np.arange(numAnts)]
        antPallets = [None          for _ in np.arange(numAnts)]
        antTorques = [None          for _ in np.arange(numAnts)]
        antSolDict = [None          for _ in np.arange(numAnts)]
        antIteDict = [None          for _ in np.arange(numAnts)]

        bestAntScore.value = initScore.value

        # stats = dict( AttractVar = 0.0, PheroVar = 0.0, AttractMean = 0.0, PheroMean = 0.0)                 
      
        best_ant = -1
        for a, _ in enumerate(ants): # ants

            # modified parameters from the greedy phase
            antPallets[a] = common.copyPallets(initPallets)              
            antTorques[a] = initTorque.value
            antSolDict[a] = dict(initSolDict)
            antIteDict[a] = dict(initItemsDict)

            antScores[a].value = initScore.value

            if mode == "Parallel": # send "antSolve" to parallel processes (ants)

                ants[a] = mp.Process( target=antSolve, args=( antPallets[a], edges, cfg, k, secBreak,\
                    antTorques[a], antSolDict[a], startTime, antScores[a], antIteDict[a], lock, n) )

                ants[a].start() # send ant

            else: # solve sequentially

                antSolve( antPallets[a], edges, cfg, k, secBreak, antTorques[a], antSolDict[a], \
                                               startTime, antScores[a], antIteDict[a], lock, n)

                # Ant System: all ants deposit pheromone
                depositPhero(antScores[a].value, iterScore.value, Attract, Phero, edges)

                # serial ant best solution update  accumsS[a]['score']
                if antScores[a].value > bestAntScore.value+0.001:
                    bestAntScore.value = antScores[a].value 
                    best_ant = a
                    improvements += 1


        if mode == "Parallel":
            # wait until time limit or all ants finish their jobs
            while time.perf_counter() - startTime <= secBreak:
                if not any(ant.is_alive() for ant in ants): # if all the ants have ended.
                    break
            else:
                # if time is over, all ants are freed
                for ant in ants:
                    ant.terminate()

            # look for the best ant solution
            for a, _ in enumerate(ants):
                ants[a].join() # get results from parallel ants

                # Ant System: all ants deposit pheromone
                depositPhero(antScores[a].value, iterScore.value, Attract, Phero, edges)

                # ant best solution update
                if antScores[a].value > bestAntScore.value+0.001:
                    bestAntScore.value = antScores[a].value 
                    best_ant = a
                    improvements += 1

        # iteration best solution update
        if bestAntScore.value > iterScore.value+0.001:
            iterScore.value = bestAntScore.value
            solDict   = dict( antSolDict[best_ant] ) 
            itemsDict = dict( antIteDict[best_ant] )
            stagnant = 0
        else:
            stagnant += 1

        iter += 1

        evaporate(Attract, Phero, edges)
  
    print(f"{improvements} improvements ({numAnts*iter} total ants).")
                
    # N = len(items)
    # Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)
    # counter = 0
    # for i, row in enumerate(Y):
    #     for j, X_ij in enumerate(row):
    #         if X_ij == 0 and pallets[i].isFeasible(items[j], limit, k, nodeTorque, solDict, lock, cfg, N, itemsDict):
    #             pallets[i].putItem( items[j], nodeTorque, solDict, lock, N, itemsDict)
    #             counter += 1
    # print(f"---> {counter} items inserted by the local search.") 

    # AttractVar  = statistics.variance(Attract)
    # PheroVar    = statistics.variance(Phero)
    # AttractMean = statistics.mean(Attract)
    # PheroMean   = statistics.mean(Phero)
    # print(f"{AttractVar:.1f}\t{AttractMean:.1f}\t{PheroVar:.3f}\t{PheroMean:.3f}")
        


if __name__ == "__main__":

    print("----- Please execute module main -----")
