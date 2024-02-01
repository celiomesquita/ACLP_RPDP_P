import common
import numpy as np
import time
import multiprocessing as mp
import os
import math

import mpShims
import mpACO
import optcgcons
import mipGRB
import mipCBC
import tabu
import grasp
import noise2
import tsp_deap

# from plots import TTT

# from py3Djanet import Packer, Item, Bin

# from py3Druiz import Packer, Item, Bin


def solveTour(scenario, instance, pi, tour, method, pallets, cfg, tourTime, folder, tipo, numOptDict, rampDistCG, afterDict, beforeDict, eta1_vol, eta2_vol):
    """
    Solves one tour
    """
    writeConsFile = False
    # writeConsFile = True  # write text files, 1 for packed each packed items

    # print(f"----- Tour {pi},", end='')
    # for node in tour.nodes:
    #     print(f" {node.ICAO}", end='')
    # print()

    # a matrix for all consolidated in the tour
    consol = [
                [ common.Item(-1, -2, 0, 0, 0., -1, -1) # an empty consolidated
                for _ in tour.nodes ]
                for _ in pallets # tour consolidated for each pallet
            ]

    k0 = len(tour.nodes)-1 # the base index on return

    # a first tour iteration to calculate the total volume to be tested for inclusion the tour nodes.
    tourVol = 0.0
    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # node max volume is updated
        _, node = common.loadNodeItems(scenario, instance, node, unattended, folder)

        tourVol += node.Vol

    # a second tour iteration solving node-by-node
    for k, node in enumerate(tour.nodes):  # solve each node sequentialy

        # the solving node time is proportional to its relative sum of candidate items volumes
        node.tLim = (node.Vol/tourVol) * tourTime

        next = tour.nodes[k0]
        if k < k0:
            next = tour.nodes[k+1]

        for i, p in enumerate(pallets):
            pallets[i].reset(cfg.numNodes)

        # solution global torque to be shared and changed by all processes concurrently
        nodeTorque = mp.Value('d', 0.0) # a multiprocessing double type variable
        nodeVol    = 0.0
        
        # initialize- the accumulated values
        if writeConsFile:  # write text files, 1 for packed each packed items
            wNodeAccum = 0.
            vNodeAccum = 0.

        # L_k destination nodes set
        unattended = [n.ID for n in tour.nodes[k+1:]]

        # load items parameters from this node and problem instance, that go to unnatended
        items, _ = common.loadNodeItems(scenario, instance, node, unattended, folder)


        if k > 0 and k < k0: # not in the base

            # load consolidated generated in the previous node
            # prevNode = tour.nodes[k-1]
            # cons = common.loadNodeCons(folder, scenario, instance, pi, prevNode, numItems )

            cons = []
            for i, _ in enumerate(pallets): # previous node consolidated
                    cons.append( consol[i][k-1] )

            # consolidated contents not destined to this point are kept on board ...
            kept = []
            cid = N
            for c in cons:
                if c.To in unattended:
                    c.ID = cid
                    kept.append(c) #... and included in the items set
                    cid += 1

            # Optimize consolidated positions to minimize CG deviation.
            # Pallets destinations are also set, according to kept on board in new positions
            # Kept P is not -2 anymore, but the pallet ID.
            if len(kept) > 0:

                optcgcons.OptCGCons(kept, pallets, k, nodeTorque)

                # N: number of items to embark
                for c in kept:
                    for i, p in enumerate(pallets):
                        if c.P == p.ID:
                            pallets[i].putConsol(c)

                            # update the consolidated of the current node "k"
                            consol[i][k].ID  = j+N
                            consol[i][k].Frm = node.ID
                            consol[i][k].To  = pallets[i].Dest[k]
                            consol[i][k].W  += c.W
                            consol[i][k].V  += c.V
                            consol[i][k].S  += c.S

                            # update the accumulated values
                            if writeConsFile:  # write text files, 1 for packed each packed items
                                wNodeAccum += c.W
                                vNodeAccum += c.V

                            tour.score += c.S
                            nodeVol    += c.V

        # set pallets destinations with items and consolidated to be delivered
        if k < k0: # except when the current node is the base on returning
            common.setPalletsDestinations(items, pallets, tour.nodes, k, unattended)
        else:
            return # skip solving because it's the base on returning

        startNodeTime = time.perf_counter()

        # to control solution items
        M = len(pallets)
        N = len(items)
        # mp.Array to be shared by multiprocess jobs
        solMatrix = mp.Array('i', [0 for _ in np.arange(N*M)] )
        mpItems   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

        # I use dict to pass by reference
        solDict   = dict(solMatrix=solMatrix)
        itemsDict = dict(mpItems=mpItems)
        modStatus = 0

        if method == "mpShims":
            mpShims.Solve(pallets, items, cfg, k, eta1_vol, eta2_vol, node.tLim, "p", nodeTorque, solDict, itemsDict, tipo) # p - parallel

        if method == "Shims":            
            mpShims.Solve(pallets, items, cfg, k, eta1_vol, eta2_vol, node.tLim, "s", nodeTorque, solDict, itemsDict, tipo)  # s - serial      

        if method == "mpACO":       
            mpACO.Solve(  pallets, items, cfg, k, eta1_vol, node.tLim, "p", nodeTorque, solDict, itemsDict) 

        if method == "ACO":       
            mpACO.Solve(  pallets, items, cfg, k, eta1_vol, node.tLim, "s", nodeTorque, solDict, itemsDict)

        if method == "TS":       
            tabu.Solve(  pallets, items, cfg, k, node.tLim, nodeTorque, solDict, itemsDict)             

        if method == "GRASP":       
            grasp.Solve( pallets, items, cfg, k, node.tLim, nodeTorque, solDict, itemsDict) 
        
        if method == "NMO":       
            noise2.Solve( pallets, items, cfg, k, node.tLim, nodeTorque, solDict, itemsDict) 

        if method == "GRB":
            modStatus, ObjBound = mipGRB.Solve( pallets, items, cfg, k, node.tLim, nodeTorque, solDict, itemsDict) 

        if method == "CBC": # new formulation
            modStatus, ObjBound = mipCBC.Solve( pallets, items, cfg, k, node.tLim, nodeTorque, solDict, itemsDict) 

        if modStatus == 2: # 2: optimal
            numOptDict["numOpt"] += 1


        nodeElapsed = time.perf_counter() - startNodeTime

        Y = np.reshape(solDict["solMatrix"], (-1, N)) # N number of items (columns)
       
        # begin ---- parallel solving the 3D packing for each pallet  
        # procs   = [None for _ in pallets]
        # packers = [None for _ in pallets]
        # counter1 = 0
        # for i, row in enumerate(Y):

        #     packers[i] = Packer()
        #     packers[i].add_bin( Bin(f'pallet{i}', pallets[i].w, pallets[i].h, pallets[i].d, pallets[i].W, i) )
            
        #     for j, X_ij in enumerate(row):
        #         if X_ij:
        #             packers[i].add_item(Item(f'item{j}', items[j].w, items[j].h, items[j].d, 0, j))
        #             counter1 += 1
            
        #     procs[i] = mp.Process( target=packers[i].pack() )
        #     procs[i].start()

        # counter2 = 0
        # for i, proc in enumerate(procs):
        #     proc.join()
        #     for bin in packers[i].bins:
        #         i = bin.ID
        #         for item in bin.unfitted_items:
        #             j = item.ID
        #             Y[i][j] = 0
        #             counter2 += 1
        #             pallets[i].popItem(items[j], nodeTorque, solDict, N, itemsDict)

        # if counter1 > 0:
        #     print(f"{100*counter2/counter1:.1f}% unfit items excluded from solution!")

        nodeElapsed2 = time.perf_counter() - startNodeTime

        # end ---- parallel solving the 3D packing for each pallet         

        # begin minRampDist
        # for p in pallets:
        #     if p.Dest[k] == next.ID:
        #         beforeDict['value'] += rampDistCG - p.D # distance from the pallet to the ramp door

        # optcgcons.minRampDist(pallets, k, tour, rampDistCG, cfg, nodeTorque)

        # for p in pallets:
        #     if p.Dest[k] == next.ID:
        #         afterDict['value'] += rampDistCG - p.D # distance from the pallet to the ramp door
        # end minRampDist

        nodeScore = 0
        torque    = 0.0

        for i, row in enumerate(Y):

            torque += 140 * pallets[i].D

            for j, X_ij in enumerate(row):
                if X_ij:
                    # mount this node "k" consolidated
                    consol[i][k].ID  = j+N
                    consol[i][k].Frm = node.ID
                    consol[i][k].To  = pallets[i].Dest[k]                    
                    consol[i][k].W += items[j].W
                    consol[i][k].V += items[j].V
                    consol[i][k].S += items[j].S

                    # totalize parameters of this solution
                    if writeConsFile: # write text files, 1 for packed each packed items
                        wNodeAccum += float(items[j].W)
                        vNodeAccum += float(items[j].V)

                    nodeScore  += items[j].S
                    nodeVol    += items[j].V

                    torque += float(items[j].W) * pallets[i].D

        tour.elapsed += nodeElapsed
        tour.elapsed2 += nodeElapsed2


        nodeVol /= cfg.volCap

        epsilon = torque/cfg.maxTorque

        # if method == "GRB":
        #     tour.score += max(ObjBound, nodeScore) # linear relaxation
        # else:

        if nodeVol > 1.0:
            nodeScore /= nodeVol

        tour.score += nodeScore

        tour.AvgVol    += nodeVol
        tour.AvgTorque += epsilon

        f = tour.score/tour.cost

        # print(f"\tnode {node.ICAO}")
        # print(f"f {f:.2f}  vol {nodeVol:.2f} epsilon {epsilon:.2f}")
        

        if writeConsFile: # write text files, 1 for packed each packed items

            consNodeT = [None for _ in pallets]        
            for i, p in enumerate(pallets):
                consNodeT[i] = consol[i][k]

            vol = vNodeAccum/cfg.volCap
            wei = wNodeAccum/cfg.weiCap

            #write consolidated contents from this node in file
            common.writeNodeCons(scenario, instance, consNodeT, pi, node, folder, epsilon, wei, vol)


# end of solveTour 

def writeAvgResults(method, scenario, line, folder):

    dirname = f"./results/{folder}/"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass  

    fname = f"{dirname}/{method}_{scenario}.avg"

    writer = open(fname,'w+') # + creates the file, if not exists

    try:
        writer.write(line)
    finally:
        writer.close()  

def writeResults(method, scenario, folder, fvalue, elapsed):

    dirname = f"./results/{folder}/"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass  

    fname = f"{dirname}/{method}_{scenario}.res"    
        
    line = f"{elapsed},{fvalue}\n"

    writer = open(fname, "a+") 
    try:
        writer.write(line)
    finally:
        writer.close() 


if __name__ == "__main__":

    plot      = False
    testing   = False
    leastCost = True # Shortest
    # leastCost = False # All K!
    iRace_testing = False

    scenarios = [2,3,4,5,6] # represent 1,2,3,4,5 in the article
    # scenarios = [7] # 7: 15-node problem
    # scenarios = [2]

    # folder = "surplus20"  # 1.2
    
    # folder = "surplus50"  # 1.5

    folder = "surplus100" # 2.0

    iRace_scenario = 2
    iRace_instance = 1

    if iRace_testing:
        # from inside folder tunning
        # irace -s scenario.txt --target-runner ./target-runner

        import sys
        eta1_vol = float(sys.argv[1])
        eta2_vol = float(sys.argv[2])
        path_items = sys.argv[3] # path to items.txt

        folder   = path_items.split("/")[7]
        iRace_scenario = int(path_items.split("/")[8][len(path_items.split("/")[8])-1])
        iRace_instance = int(path_items.split("/")[9][len(path_items.split("/")[9])-1])

        scenarios = [iRace_scenario]

    # Apply iRace results
    eta1_vol, eta2_vol = 0.8621, 1.0539

    if folder == "surplus50":
        eta1_vol, eta2_vol = 0.9199, 1.1399

    if folder == "surplus100":
        eta1_vol, eta2_vol = 0.9617, 1.5706

    # timeLimit = 240
    # timeLimit = 1200
    # timeLimit = 2400
    timeLimit = 3600

    # method = "GRB"
    # method = "CBC"
    method = "Shims"
    # method = "mpShims"
    # method = "mpACO"
    # method = "ACO"
    # method = "TS"
    # method = "GRASP"
    # method = "NMO"

    print(f"timeLimit:{timeLimit} folder: {folder} method: {method}")

    # tipo = "KP"
    tipo = "FFD"

    distances_file = "./params/distances7.txt"
    if scenarios[0] == 7:
        distances_file = "./params/distances15.txt"

    dists, _ = common.loadDistances(distances_file) # dists, cities
    costs = [[0.0 for _ in dists] for _ in dists]

    for scenario in scenarios:

        if testing:
            instances = [1]

        if iRace_testing:
            instances = [iRace_instance]

        cfg = common.Config(scenario)

        instances = [1,2,3,4,5,6,7]

        print(f"{cfg.numNodes} nodes")

        if cfg.numNodes > 7: # Shims validation
            instances = [1]

        for i, cols in enumerate(dists):
            for j, dist in enumerate(cols):
                costs[i][j] = cfg.kmCost*dist

        pallets, rampDistCG = common.loadPallets(cfg)

        # pallets capacity
        cfg.weiCap = 0
        cfg.volCap = 0
        for p in pallets:
            cfg.weiCap += p.W
            cfg.volCap += p.V

        # smaller aircrafts may have a payload lower than pallets capacity
        if cfg.weiCap > cfg.payload:
            cfg.weiCap = cfg.payload

        perc = 1.0
        if cfg.numNodes > 3 and cfg.numNodes <= 7:
            perc = 0.25 # discard the worst tours

        # time limit per tour
        tourTime = timeLimit/common.factorial(cfg.numNodes)

        if leastCost: # the 2 shortest tours
            tourTime = timeLimit/2

        instanceTime = 0.
        instanceTime2 = 0. # with 3D packing
        instanceSC   = 0. # score/cost relation
        leastSC      = 0.
        worstTime    = 0

        numOptDict = {"numOpt":0}
        afterDict  = {"value":0.}
        beforeDict = {"value":0.}


        for inst in instances:  

            bestSC = 0. # maximum score/cost relation
            # leastSC = 0. # minimum cost score/cost relation
            bestAV = 0.
            bestAT = 0.


            if cfg.numNodes <= 7:
                # permutation of nodes - TSP solution with all tours
                tours = common.getTours(cfg.numNodes-1, costs, perc)
            else:
                # TSP solution with heuristics
                tours = tsp_deap.getTours(distances_file, cfg.numNodes)

            # print( len(tours), len(tours[0].nodes) )

            # selects the best tour
            searchTime = 0
            searchTime2 = 0 # with 3D packing
            bestTourID = -1
            bestTour = []
            for pi, tour in enumerate(tours):

                if leastCost and pi >= 2: # the first two shortest tours
                    break

                tour.elapsed = 0
                tour.elapsed2 = 0 # with 3D packing
                tour.score   = 0.0
                tour.AvgVol  = 0.0

                solveTour(scenario, inst, pi, tour, method, pallets, cfg, tourTime, folder, tipo, numOptDict, rampDistCG, afterDict, beforeDict, eta1_vol, eta2_vol)

                # if not iRace_testing:
                #     print(f"\tTour elapsed: {tour.elapsed:.1f}s")

                # the tour cost is increased by the average torque deviation, limited to 5%
                tour.AvgTorque /= cfg.numNodes
                tour.cost *= ( 1.0 + abs(tour.AvgTorque)/20.0 )

                searchTime += tour.elapsed
                searchTime2 += tour.elapsed2

                tourSC = tour.score / tour.cost

                tour.AvgVol /= cfg.numNodes

                # best tour parameters
                if tourSC > bestSC:
                    bestSC = tourSC
                    bestAV = tour.AvgVol
                    bestAT = tour.AvgTorque
                    bestTourID = pi
                    bestTour = tour.nodes

                if tour.elapsed > worstTime:
                    worstTime = tour.elapsed

                if pi == 0:
                    leastSC += tourSC
            
            instanceTime  += searchTime
            instanceTime2 += searchTime2
            instanceSC    += bestSC

            # for plotting
            # writeResults(method, scenario, folder, f"{instanceSC:.2f}", f"{tour.elapsed:.2f}")


            numInst = float(len(instances))

            numOptDict["numOpt"] /= numInst
            numOptDict["numOpt"] /= float(cfg.numNodes)
            numOptDict["numOpt"] /= float(len(tours))

            afterDict["value"] /= numInst
            afterDict["value"] /= float(cfg.numNodes)
            afterDict["value"] /= float(len(tours))

            beforeDict["value"] /= numInst
            beforeDict["value"] /= float(cfg.numNodes)
            beforeDict["value"] /= float(len(tours))

            percent = 0.0
            if beforeDict["value"] > 0:
                percent = 100.0*( afterDict["value"] - beforeDict["value"]  ) / beforeDict["value"]   

            avgTime  = math.ceil(instanceTime/numInst)
            avgTime2 = math.ceil(instanceTime2/numInst)

        icaos = []
        for n in bestTour:
            icaos.append(n.ICAO)

        icaos = tsp_deap.rotate(icaos, "GRU") # GRU is the base

        sbest_tour = tsp_deap.list_to_string(icaos)

        # list the Shims best tour as an ICAO list
        # origin = icaos[0]
        # sbest_tour = f"{origin} "
        # prev = icaos[0]
        # for j, icao in enumerate(icaos):
        #     if j > 0:
        #         sbest_tour += f"{icao} "
        #         prev = icao

        # # list the GA best tour as an ICAO list
        # origin = tours[0].nodes[0]
        # shortestTour = f"{origin.ICAO} "
        # prev = tours[0].nodes[0]
        # for j, node in enumerate(tours[0].nodes):
        #     if j > 0:
        #         shortestTour += f"{node.ICAO} "
        #         prev = node
        # shortestTour += f"{origin.ICAO}"

        if not iRace_testing:

            str = f"{leastSC/numInst:.2f}\t {instanceSC/numInst:.2f}\t {avgTime:.0f}\t {avgTime2:.0f}\t {worstTime:.1f}\t {bestAV:.2f}\t {bestAT:.2f}\t {numOptDict['numOpt']:.1f}\t {beforeDict['value']:.1f} & {afterDict['value']:.1f} & {percent:.1f}\n"
            # instances average
            writeAvgResults(method, scenario, str, folder)

            print(f"{str}")
            # print(f"{folder}")
            print(f"{len(tours)} tours")
            # print(f"tourTime: {tourTime} \t leastCost = {leastCost}")
            # print(f"eta1_vol: {eta1_vol:.2f}")
            # print(f"Before:\t{beforeDict['value']:.1f}") 
            # print(f"After:\t{afterDict['value']:.1f}")
            # print(f"% of optima: {numOptDict['numOpt']:.2f}")
            # print(f"{method}")
            # print(f"    best: {sbest_tour}")
            # print(f"shortest: {shortestTour}")

        else:
            print(-1*instanceSC/numInst) # -1: iRace minimizes a cost value

    #"""
