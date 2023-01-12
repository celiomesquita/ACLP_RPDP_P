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

    def isFeasible(self, item, k, solTorque, cfg, lock): # check constraints

        if item.To != self.Pallet.Dests[k]:
            return False

        if self.Pallet.PCV + self.SCV + item.V > self.Pallet.V:
            return False

        deltaTau = float(item.W) * float(self.Pallet.D)
        ret = True
        with lock:
            newTorque = abs(solTorque.value + self.SCT + deltaTau)

            if newTorque > abs(solTorque.value + self.SCT):
                if newTorque > cfg.maxTorque:
                    ret = False

        return ret        

# create a set of shims for this pallet and selects the best shims
def getBestShims(pallet, items, k, solTorque, solItems, lock, cfg, surplus):

    maxVol = pallet.V * surplus

    vol = 0.
    whip = []
   
    with lock:
        for item in items:
            # not consolidated (-1) and not allocated in any pallet
            if item.P == -1 and solItems[item.ID] == -1: 
                vol += item.V
                whip.append(item)
                if vol > maxVol:
                    break

    # create the first shim
    sh = Shims(pallet, len(whip))
    Set = [sh]

    # First Fit Decrease - equivalente ao KP, mas mais rápido
    whip.sort(key=lambda x: abs(x.V), reverse=True)

    for whipPos, item in enumerate(whip):
        newShims = True
        for sh in Set:
            if sh.isFeasible(item, k, solTorque, cfg, lock):
                sh.putItem(item, whipPos)
                newShims = False
                break

        if newShims:
            sh = Shims(pallet, len(whip)) # create a new Shim
            if sh.isFeasible(item, k, solTorque, cfg, lock):
                sh.putItem(item, whipPos)
            Set.append(sh)
                
    bestScore = 0
    bestIndex = 0
    for i, shims in enumerate(Set):
        if shims.SCS > bestScore:
            bestScore = shims.SCS
            bestIndex = i

    for item in Set[bestIndex].Items:
        if item != None:
            pallet.putItem(item, solTorque, solItems, lock)


def Solve(pallets, items, cfg, k, limit, secBreak, mode, solTorque, dictItems): # items include kept on board

    # solTorque was first updated when consolidaded were put in the pallets

    solItems = common.copySolItems(dictItems["solItems"])

    if mode == "p":
        mode = "Parallel"
    else:
        mode = "Serial"

    print(f"\n{mode} Shims for ACLP+RPDP")

    # surplus = math.exp(-limit) + 0.9
    # surplus = 1. + 2. * (1. - limit)
    surplus = (2. - limit)

    print(f"surplus: {surplus:.2f}")

    lock  = mp.Lock()

    procs = [None for _ in pallets] # each pallets has its own process

    # sort ascendent by CG distance
    pallets.sort(key=lambda x: abs(x.D), reverse=False)


    if mode == "p":

        start = time.time()

        # parallel greedy phase
        for i, _ in enumerate(procs):
            procs[i] = mp.Process( target=common.fillPallet, args=( pallets[i], items, k, solTorque, solItems, lock, cfg, limit) )
            time.sleep(0.001)
            procs[i].start()
        
        for proc in procs:
            proc.join()

        # parallel shims phase
        for i, _ in enumerate(procs):
            procs[i] = mp.Process( target=getBestShims, args=( pallets[i], items, k, solTorque, solItems, lock, cfg, surplus) )
            procs[i].start()
                
        while time.time() - start <= secBreak:
            if not any(p.is_alive() for p in procs):
                # All the processes are done, break now.
                break
        else:
            # We only enter this if we didn't 'break' above.
            print("timed out, killing all processes")
            for p in procs:
                p.terminate()
                p.join()

    else: # serial
        initScore = 0.0
        for i, _ in enumerate(pallets):
            common.fillPallet(  pallets[i], items, k, solTorque, solItems, lock, cfg, limit) 
            initScore += pallets[i].PCS
            getBestShims(pallets[i], items, k, solTorque, solItems, lock, cfg, surplus)

        print(f"Greedy initial score {initScore}")
               

    dictItems["solItems"] = common.copySolItems(solItems)


if __name__ == "__main__":

    print("----- Please execute module main -----")
