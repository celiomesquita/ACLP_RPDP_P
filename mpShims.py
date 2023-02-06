import numpy as np
import multiprocessing as mp
import time
import common

# A Shims fits in a pallet slack
class Shims(object):
    def __init__(self, pallet, whipLen):
        self.Pallet = pallet # this shims is one of the possibles for this pallet
        self.SCW    = 0      # shims current weight (must fit the slack)
        self.SCV    = 0.0    # shims current volume (must fit the slack)
        self.SCS    = 0      # shims current score
        self.SCT    = 0.     # shims current torque
        self.Items  = [None for _ in np.arange(whipLen)]

    def putItem(self, item, whipPos):
                 
            self.SCW += item.W
            self.SCV += item.V
            self.SCS += item.S
            self.SCT += float(item.W) * float(self.Pallet.D)
            self.Items[whipPos] = item

    def isFeasible(self, item, k, nodeTorque, cfg, lock): # check constraints

        if item.To != self.Pallet.Dest[k]:
            return False

        if self.Pallet.PCV + self.SCV + item.V > self.Pallet.V:
            return False

        deltaTau = float(item.W) * float(self.Pallet.D)
        ret = True
        with lock:
            newTorque = abs(nodeTorque.value + self.SCT + deltaTau)

            if newTorque > abs(nodeTorque.value + self.SCT) and newTorque > cfg.maxTorque:
                    ret = False
        return ret

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, k, nodeTorque, solDict, cfg, surplus, itemsDict, lock):

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

    # First Fit Decrease - faster than KP
    whip.sort(key=lambda x: abs(x.V), reverse=True)

    for whipPos, item in enumerate(whip):
        newShims = True
        for sh in Set:
            if sh.isFeasible(item, k, nodeTorque, cfg, lock):
                sh.putItem(item, whipPos)
                newShims = False
                break

        if newShims:
            sh = Shims(pallet, len(whip)) # create a new Shim
            if sh.isFeasible(item, k, nodeTorque, cfg, lock):
                sh.putItem(item, whipPos)
            Set.append(sh)
                
    bestScore = 0
    bestIndex = 0
    for i, shims in enumerate(Set):
        if shims.SCS > bestScore:
            bestScore = shims.SCS
            bestIndex = i

    for item in Set[bestIndex].Items:
        if item != None and pallet.isFeasible(item, 1.0, k, nodeTorque,  cfg, itemsDict, lock):
            pallet.putItem(item, nodeTorque, solDict, N, itemsDict, lock)


def Solve(pallets, items, cfg, k, threshold, secBreak, mode, nodeTorque, solDict, itemsDict):

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    print(f"\n{mode} Shims for ACLP+RPDP")

    surplus = (2. - threshold)

    # print(f"surplus: {surplus:.2f}")

    lock  = mp.Lock()

    # vol = 0.0
    # for p in pallets:
    #     vol += p.PCV
    # print(f"1 vol: {vol:.2f}")

    if mode == "Parallel":
    
        procs = [None for _ in pallets] # each pallets has its own process

        # parallel greedy phase
        for i, _ in enumerate(pallets):
            procs[i] = mp.Process( target=common.fillPallet, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, threshold, itemsDict, lock) )
            time.sleep(0.001)
            procs[i].start()
        
        for proc in procs:
            proc.join()

        # parallel shims phase
        for i, p in enumerate(pallets):
            procs[i] = mp.Process( target=getBestShims, args=( pallets[i], items, k,\
                 nodeTorque, solDict, cfg, surplus, itemsDict, lock) )
            time.sleep(0.001)                 
            procs[i].start()
                
        for p in procs:
            p.join()

    else: # serial
        # sort ascendent by CG distance
        pallets.sort(key=lambda x: abs(x.D), reverse=False) 

        for i, _ in enumerate(pallets):
            common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, threshold, itemsDict, lock) 

        # vol = 0.0
        # for p in pallets:
        #     vol += p.PCV
        # print(f"2 vol: {vol:.2f}")

        for i, _ in enumerate(pallets):
            getBestShims(      pallets[i], items, k, nodeTorque, solDict, cfg, surplus,   itemsDict, lock)


    # pallets.sort(key=lambda x: abs(x.ID)) 

    # vol = 0.0
    # for p in pallets:
    #     vol += p.PCV
    # print(f"3 vol: {vol:.2f}")

    # local search
    counter = 0
    for i, _ in enumerate(pallets):
        counter += common.fillPallet( pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock) 
    print(f"---> {counter} items inserted by the local search.")

    # vol = 0.0
    # for p in pallets:
    #     vol += p.PCV
    # print(f"4 vol: {vol:.2f}")


if __name__ == "__main__":

    print("----- Please execute module main -----")
