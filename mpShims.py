import numpy as np
import multiprocessing as mp
import time
import math

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
            newTorque = abs(nodeTorque.value + self.SCT + deltaTau)

            if newTorque > abs(nodeTorque.value + self.SCT) and newTorque > cfg.maxTorque:
                    ret = False
        return ret

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, k, nodeTorque, solDict, cfg, surplus, itemsDict, lock, tipo):

    maxVol = pallet.V * surplus

    vol = 0.
    whip = []
    N = len(items)
    i = pallet.ID
   
    with lock:   
        for item in items:
            j = item.ID
            if itemsDict["mpItems"][j] == 0:
                vol += item.V
                whip.append(item)
                if vol > maxVol:
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

            for w, item in enumerate(whip):
                if indexes[w] == 1 and pallet.isFeasible(item, 1.0, k, nodeTorque, cfg, itemsDict, lock):
                    pallet.putItem(item, nodeTorque, solDict, N, itemsDict, lock)

    if tipo == "FFD":
        # First Fit Decrease - faster than KP
        whip.sort(key=lambda x: abs(x.V), reverse=True)

        for w, item in enumerate(whip):
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


def Solve(pallets, items, cfg, k, threshold, secBreak, mode, nodeTorque, solDict, itemsDict, tipo):

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    print(f"\n{mode} Shims for ACLP+RPDP")

    surplus = 1.0 + 3*(1.0-threshold)

    lock  = mp.Lock()

    if mode == "Parallel":
    
        procs = [None for _ in pallets] # each pallets has its own process

        # parallel greedy phase
        for i, _ in enumerate(pallets):
            procs[i] = mp.Process( target=common.fillPallet, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, threshold, itemsDict, lock, 1.1) ) # 1.1 torque surplus
            time.sleep(0.001)
            procs[i].start()
        
        for proc in procs:
            proc.join()

        optcgcons.OptCG(pallets, k, nodeTorque)

        # parallel shims phase
        for i, p in enumerate(pallets):
            procs[i] = mp.Process( target=getBestShims, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, surplus, itemsDict, lock, tipo) )
            time.sleep(0.001)                 
            procs[i].start()
                
        for p in procs:
            p.join()

    else: # serial
        # sort ascendent by CG distance
        pallets.sort(key=lambda x: abs(x.D), reverse=False) 
        counter = 0
        for i, _ in enumerate(pallets):                                                 # torque surplus = 1.3
            common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, threshold, itemsDict, lock, 1.3) 
            optcgcons.OptCG(pallets, k, nodeTorque)
            getBestShims(      pallets[i], items, k, nodeTorque, solDict, cfg, surplus,   itemsDict, lock, tipo)
            counter += common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock) 
        print(f"---> {counter} items inserted by the local search.")


if __name__ == "__main__":

    print("----- Please execute module main -----")
