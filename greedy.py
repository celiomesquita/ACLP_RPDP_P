
import common
import multiprocessing as mp

def Solve(pallets, items, cfg, pi, k, secBreak, nodeTorque, solDict, itemsDict):

    N = len(items)
    M = len(pallets)

    lock  = mp.Lock() # for use in parallel mode

    for i, _ in enumerate(pallets): #                                     vol   
        common.fillPallet(pallets[i], items, k, nodeTorque, solDict, cfg, 1.0, itemsDict, lock)


if __name__ == "__main__":

    print("----- Please execute the main.py file -------------")