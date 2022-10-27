import numpy as np
from timeit import default_timer as timer

import methods as mjit

def factorial(x):
    result = 1
    for i in range(x):
        result *= i+1
    return result

def permutations(n):
    fac = factorial(n)
    a = np.zeros((fac, n), np.uint32) # no jit
    f = 1
    for m in range(2, n+1):
        b = a[:f, n-m+1:]      # the block of permutations of range(m-1)
        for i in range(1, m):
            a[i*f:(i+1)*f, n-m] = i
            a[i*f:(i+1)*f, n-m+1:] = b + (b >= i)
        b += 1
        f *= m
    return a

def getTours(A, costs, threshold):

    p = permutations(A)

    mcosts = np.full(len(p), 0.)
    toursInt = [[0 for _ in range(len(p[0])+2)] for _ in range(len(p))]

    for i, row in enumerate(p):
        for j, col in enumerate(row):
            toursInt[i][j+1] = col+1

    minCost = 9999999999999.

    for i, tour in enumerate(toursInt):
        for j, node in enumerate(tour):
            if j > 0:
                prev = tour[j-1]
                mcosts[i] += costs[node][prev]

        if mcosts[i] < minCost:
            minCost = mcosts[i]

    for i, cost in enumerate(mcosts):
        if cost > (1+threshold) * minCost:
            toursInt.pop(i)

    tours = [[None for _ in range(len(toursInt[0]))] for _ in range(len(toursInt))]

    for i, tour in enumerate(toursInt):
        for j, node in enumerate(tour):
            tours[i][j] = mjit.Node(node, 0.)

    return tours


if __name__ == "__main__":


    # t0 = timer()
    dists = mjit.loadDistances()

    costs = [[0.0 for _ in dists] for _ in dists]

    scenario = 3

    cfg = mjit.Config(scenario)

    for i, cols in enumerate(dists):
        for j, value in enumerate(cols):
            costs[i][j] = cfg.kmCost*value

    # tours IDs
    tours = getTours(cfg.numNodes-1, costs, 0.05)

     # t1 = timer()

    # print(t1-t0)

    
