
import common
from time import time
import math
import numpy as np
from multiprocessing import Queue, Process
import common

RNG = np.random.default_rng()

# ProbAccept is the Noising Method acceptance method.
# This feature prevents the method from becoming stuck at a local optima.
def ProbAccept(newScore, oldScore, r):
	delta = float(newScore-oldScore) / float(oldScore)
	return delta + (2*r*RNG.random() - r)  > 0 

def Solve( pallets, items, startTime, cfg, k):  # items include kept on board

    print("\nNoising Method Optimization for ACLP+RPDP")

    Gbest  = common.Solution(pallets, items, 1.0, cfg, k, False) # False is Greedy
    Gprime = common.Solution(pallets, items, 0.7, cfg, k, False)

    r_max = 1 - (Gbest.S - Gprime.S)/Gbest.S # maximum initial noise

    numTrials = math.ceil(len(Gbest.Nbhood)/50)

    numIter = math.ceil(len(Gbest.Edges)/30)

    print(numTrials,numIter,math.ceil(numTrials/numIter))

    step = r_max/(numTrials/numIter-1)
    r = r_max

    trial = 0
    while trial < numTrials:

        # restart: copy the best so far local iteration
        Glocal = Gbest
        
        for _ in range(numIter):

            trial += 1

            if ((time() - startTime) > common.SEC_BREAK):
                trial = numTrials
                break

            Gprime = Glocal

            oldScore = Gprime.S

            if Gprime.Transform(k, cfg) and ProbAccept(Gprime.S, oldScore, r):
                Glocal = Gprime
            
        for ce in Glocal.Nbhood: # try to improve
            if Glocal.isFeasible(ce, 1.0, cfg, k): # Check all constraints
                Glocal.AppendEdge(ce)
                Glocal.Nbhood.remove(ce)

        if Glocal.S > Gbest.S:
            Gbest = Glocal

        r -= step

    # decision matrix for which items will be put in which pallet
    X = [[0 for _ in np.arange(len(items))] for _ in np.arange(len(pallets))]
    for e in Gbest.Edges:
        X[e.Pallet.ID][e.Item.ID] = 1

    return X


if __name__ == "__main__":

    print("----- Please execute the main py file -------------")