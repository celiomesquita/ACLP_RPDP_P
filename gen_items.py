
import math
import random
import common
import os
import numpy as np

RNG = np.random.default_rng()

def randomProportionalSelection(values, maxVal):

    i = 0
    while True:
        j = int(5 * RNG.random())
        if RNG.random() < (float(values[j]) / float(maxVal)):
            i = j
            break

    if i == 0:
        return 10,20
    if i == 1:
        return 20,40
    if i == 2:
        return 40,80
    if i == 3:
        return 80,200
    if i == 4:
        return 200,340


def nextIsBase(node, nodes):

    k = nodes.index(node)
    if k < len(nodes) and nodes[k+1] == 0:
        return True
    return False

def genItems(sce, inst, surplus):

    cfg = common.Config(sce)

    dirname = f"./surplus{math.ceil((surplus-1)*100)}/scenario_{sce}/instance_{inst}"
    try:
        os.makedirs(dirname)
    except FileExistsError:
        pass

    fname = "%s/items.txt" % (dirname)

    lines = ""
    x = 0 # to count the number of items created
    pallets, _ = common.loadPallets(cfg)
    # pallets capacity
    limit = 0
    for p in pallets:
        limit += p.V

    limit *= surplus # % over the capacities

    totals = [0.0 for _ in range(cfg.numNodes)]

    nodes = list(range(cfg.numNodes))
    nodes.append(0) # back to the the base

    dests = nodes

    values = [23, 22, 24, 23, 8]

    for node in nodes:

        lim = limit
        if nextIsBase(node, nodes):
            lim = surplus*limit

        while totals[node] < lim:

            to = np.random.choice(dests, 1)

            if node != to:

                if nextIsBase(to, nodes):
                    lim = surplus*limit
                
                low, high = randomProportionalSelection(values, 24)

                w = random.randrange(low, high)
                v = w/random.randrange(148, 344)

                totals[node] += v

                s = math.floor(0.5 + 100 * (1 - math.log10( random.randrange(1, 10) ) ))
                lines += "%d %d %.3f %d %d\n" % (w, s, v, node, to)
                x += 1

    writer = open(fname, "w+") 
    try:
        writer.write(lines)
    finally:
        writer.close()

    print("-----Instance %d - %d items in %d nodes generated with success." % (inst, x, cfg.numNodes))
            

if __name__ == "__main__":

    # scenarios = [1,2,3,4,5,6]
    scenarios = [7,8,9,10,11,12,13,14]

    instances = [2,3,4,5,6,7]
    # instances = [1]

    # surplus = 1.2
    # surplus = 1.5
    surplus = 2.0

    for sce in scenarios:

        cfg = common.Config(sce)

        print("Scenario %d" % (sce))
    
        for inst in instances:
            genItems(sce, inst, surplus)

