

import methods
from time import time
import math
from os import path
import numpy as np

import methods
import optcgcons
import aco
import mips
import shims

# preparations to test acoSolve
if __name__ == "__main__":

    two_up = path.abspath(path.join(__file__, "../..")) + "/data"

    # FOR TESTING ONLY
    # DO NOT CHANGE THESE PARAMETERS
    scenario = 1
    instance = 1
    tix      = 0  # tour index in the permutations
    tour     = [0, 1, 2, 0]

    BestScores = [0]*len(tour)

    for node in tour:

        method = "GRB6s"
        # method = "GRB60s"
        # method = "GRB10m"

        cfg = methods.Config(scenario)

        pallets = methods.loadPallets(cfg.aircraft.size, two_up)

        print("len(pallets)", len(pallets))

        nix = tour.index(node)  # node index in the tour
        unattended = tour[nix+1:]

        print("len(unattended)", len(unattended))

        items = methods.loadNodeItems(scenario, instance, node, unattended, two_up)

        numItems = len(items)
        print("numItems", numItems)

        # pallets capacity
        weiCap = 0
        volCap = 0
        for p in pallets:
            weiCap += p.W
            volCap += p.V

        # smaller aircrafts may have a payload lower than pallets capacity
        if weiCap > cfg.aircraft.payload:
            weiCap = cfg.aircraft.payload

        # for maximum torque constraint
        maxTorque = cfg.aircraft.maxTorque

        # consolidated generated in node 0, to be dealt with in node 1
        cons = []

        if nix > 0:

            # load consolidated generated in the previous node
            cons = methods.loadNodeCons( scenario, instance, tix, 0, two_up, numItems )


        print("-----Loaded from tour %d %s -----" % (tix, methods.cities[0]))
        for c in cons:
            print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                c.ID, c.W, c.S, c.V, methods.cities[c.Frm], methods.cities[c.To]))

        # consolidated contents not destined to this point are kept on board
        kept = []
        id = len(items)
        for c in cons:
            if c.To in unattended:
                c.ID = id
                c.Frm = node  # now the consolidated from the previous node depart from this node
                kept.append(c)
                id += 1

        # optimize consolidated positions to minimize CG deviation
        if len(kept) > 0:
            optcgcons.OptCGCons(kept, pallets, maxTorque, "GRB")

        print("\n-----Consolidated contents from tour %d %s kept on board -----" %
            (tix, methods.cities[0]))

        # kept on board, with new pallets positions, is appended to items
        for c in kept:
            items.append(c)

            print("%d\t%d\t%d\t%.1f\t%s\t%s" % (
                c.ID, c.W, c.S, c.V, methods.cities[c.Frm], methods.cities[c.To]))

        # non-defined pallets destinations are set according to weight demand on next nodes
        methods.setPalletsDestinations(items, pallets)

        for p in pallets:
            print(f"{p.To} ", end='')
        print()

        startTime = time()

        numOpts = 0
        opt = 0

        if method == "GRB6s":
            E, opt =  mips.Solve("GRB", pallets, items, tix, node, maxTorque, 6)

        if method == "GRB60s":
            E, opt =  mips.Solve("GRB", pallets, items, tix, node, maxTorque, 60)

        if method == "GRB10m":
            E, opt =  mips.Solve("GRB", pallets, items, tix, node, maxTorque, 600)

        if method == "GRB2h":
            E, opt =  mips.Solve("GRB", pallets, items, tix, node, maxTorque, 7200)

        if opt > 0: # may come negative
            numOpts += opt

        if method == "Shims":
            E = shims.Solve(pallets, items, maxTorque)    

        if method == "ACO":
            E = aco.teamSolve(pallets, items, maxTorque, startTime)  

        elapsed2 = time() - startTime

        # print the solution
        if len(E) > 0:

            numItems = len(E[0])
            numPallets = len(E)
            newCons = []

            wTotAccum = 0
            vTotAccum = 0
            sTotAccum = 0
            tTotAccum = 0
            for i in np.arange(numPallets):

                tTotAccum += float(140) * pallets[i].D
                con = methods.Item(0, 1, 0, 0, 0, node, -1)

                print(f"{i}: ", end='')

                for j in np.arange(numItems):

                    if E[i][j] == 1:

                        con.W += items[j].W
                        con.V += items[j].V
                        con.S += items[j].S
                        if con.To == -1:
                            con.To = items[j].To

                        wTotAccum += items[j].W
                        vTotAccum += items[j].V
                        sTotAccum += items[j].S
                        tTotAccum += float(items[j].W) * pallets[i].D

                        print(f"{items[j].W} ", end='')
                print()
                if con.W > 0:
                    newCons.append(con)

            # write consolidated contents from this node in file
            methods.writeNodeCons(scenario, instance, newCons, two_up, tix, node)

            elapsed1 = math.ceil(1000*elapsed1)  # milliseconds
            elapsed2 = math.ceil(1000*elapsed2)  # milliseconds
            elepsedMin = (elapsed1 + elapsed2)/60000

            print(f"Node {node} with {method}")
            print(f"Best score: {BestScore}, this score: {sTotAccum}")
            print(f"Rel. score: {sTotAccum/BestScore:.2f} | Rel. torque: {tTotAccum/maxTorque:.2f} | elapsed: {elapsed1} + {elapsed2} = {elapsed1 + elapsed2} ms ({elepsedMin:.1f}min)")

