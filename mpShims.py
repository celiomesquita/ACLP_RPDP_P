import numpy as np
import multiprocessing as mp
import time
import math
import os

import knapsack as kp
import common
import optcgcons

# A Shims fits in a pallet slack
class Shims(object):
    def __init__(self, pallet, whipLen):
        self.Pallet = pallet # this shims is one of the possibles for this pallet
        self.SCW    = 0      # shims current weight (must fit the slack)
        self.SCV    = 0.0    # shims current volume (must fit the slack)
        self.SCS    = 0      # shims current score
        self.SCT    = 0.     # shims current torque
        self.Items  = [None for _ in np.arange(whipLen)]

    def putItem(self, item, w):
                 
            self.SCW += item.W
            self.SCV += item.V
            self.SCS += item.S
            self.SCT += float(item.W) * float(self.Pallet.D)
            self.Items[w] = item

    def isFeasible(self, item, k, nodeTorque, cfg, lock): # check constraints

        if item.To != self.Pallet.Dest[k]:
            return False

        if self.Pallet.PCV + self.SCV + item.V > self.Pallet.V:
            return False

        if self.Pallet.PCW + self.SCW + item.W > self.Pallet.W:
            return False

        deltaTau = float(item.W) * float(self.Pallet.D)
        ret = True
        with lock:
            newTorque = nodeTorque.value + self.SCT + deltaTau

            """
            Infeasible:
            If the torque is increasing positively and is bigger  than max
            If the torque is increasing negatively and is smaller than -max
            """
            if nodeTorque.value + self.SCT > 0 and newTorque > 0 and nodeTorque.value + self.SCT < newTorque and newTorque  > cfg.maxTorque:
                ret = False
                
            if nodeTorque.value + self.SCT < 0 and newTorque < 0 and newTorque < nodeTorque.value + self.SCT and newTorque  < -cfg.maxTorque:
                ret = False

        return ret

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, k, nodeTorque, solDict, cfg, eta2_vol, itemsDict, lock, tipo, startTime, secBreak):

    vol = 0.
    whip = []
    N = len(items)
    i = pallet.ID
   
   # create the whip
    with lock:   
        for item in items:
            j = item.ID
            if itemsDict["mpItems"][j] == 0:
                vol += item.V
                whip.append(item)
                if vol > pallet.V * eta2_vol:
                    break

    # create the first shim
    sh = Shims(pallet, len(whip))
    Set = [sh]

    if tipo == "KP":
        capacity = math.floor((pallet.V - pallet.PCV)*100)
        if capacity > 0:
            volumes = [0]*len(whip)
            scores  = [0]*len(whip)
            for w, item in enumerate(whip):
                volumes[w] = math.floor(item.V*100)
                scores[w]  = item.S
                    
            indexes = kp.Solve(capacity, volumes, scores)

            for i in indexes:
                item = whip[i]
                if pallet.isFeasible(item, 1.0, k, nodeTorque,  cfg, itemsDict, lock):
                    pallet.putItem(item, nodeTorque, solDict, N, itemsDict, lock)

    if tipo == "FFD":
        # First Fit Decrease - faster than KP
        whip.sort(key=lambda x: abs(x.V), reverse=True)

        for w, item in enumerate(whip):

            if ((time.perf_counter() - startTime) > secBreak):
                break 

            newShims = True
            for sh in Set:
                if sh.isFeasible(item, k, nodeTorque, cfg, lock):
                    sh.putItem(item, w)
                    newShims = False
                    break

            if newShims:
                sh = Shims(pallet, len(whip)) # create a new Shim
                if sh.isFeasible(item, k, nodeTorque, cfg, lock):
                    sh.putItem(item, w)
                Set.append(sh)
        # select the best Shim
        bestScore = 0
        bestIndex = 0
        for i, shims in enumerate(Set):
            if shims.SCS > bestScore:
                bestScore = shims.SCS
                bestIndex = i
        # put the best Shim in the solution
        for item in Set[bestIndex].Items:
            if item != None and pallet.isFeasible(item, 1.0, k, nodeTorque,  cfg, itemsDict, lock):
                pallet.putItem(item, nodeTorque, solDict, N, itemsDict, lock)

#                                    eta 1    eta 2
def Solve(pallets, items, cfg, k, eta1_vol, eta2_vol, secBreak, mode, nodeTorque, solDict, itemsDict, tipo):

    startTime = time.perf_counter()

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    lock  = mp.Lock()
    counter = 0

    for i, _ in enumerate(pallets):
        common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, eta1_vol, itemsDict, lock, ts=2.1)

    optcgcons.minCGdev(pallets, k, nodeTorque, cfg)

    # best: optcgcons before
    # 2.5 33.6
    # 2.3 32.7
    # 2.1 34.1
    # 1.9 33.2
    # 1.7 32.0
    # 1.5 31.5


    if mode == "Parallel":
    
        procs = [None for _ in pallets] # each pallets has its own process

        # parallel shims phase
        for i, p in enumerate(pallets):
            procs[i] = mp.Process( target=getBestShims, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, eta2_vol, itemsDict, lock, tipo, startTime, secBreak) )
            time.sleep(0.001)                 
            procs[i].start()
                
        for p in procs:
            p.join()

    else: # serial
        for i, _ in enumerate(pallets):

            # get the best Shims for the pallet
            getBestShims( pallets[i], items, k, nodeTorque, solDict, cfg, eta2_vol, itemsDict, lock, tipo, startTime, secBreak)


    # try to complete the pallet
    for i, _ in enumerate(pallets):
        counter += common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock)

    # if counter > 0:
    #     print(f"----->{mode}: {counter} items inserted in post local search.")


if __name__ == "__main__":

    print("----- Please execute module main -----")



