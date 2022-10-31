
import methods
from os import path
from time import time
import math
from mip import Model, xsum, minimize, BINARY, CBC, OptimizationStatus

# optimize consolidated positions to minimize CG deviation
def OptCGCons(kept, pallets, maxTorque, method, k):

    KeptRange    = range(len(kept))
    PalletsRange = range(len(pallets))

    mod = Model(solver_name=method)

    X = [ [ mod.add_var(name="X({},{})".format(i, j), var_type=BINARY) for j in KeptRange ] for i in PalletsRange ]      

    torque1 = xsum( X[i][j] * (kept[j].W *    pallets[i].D) for i in PalletsRange for j in KeptRange ) 
    torque2 = xsum( X[i][j] * (kept[j].W * -1*pallets[i].D) for i in PalletsRange for j in KeptRange ) 

    mod.objective = minimize( torque1 + torque2 )  

    for j in KeptRange:
        # all consolidated must be embarked
        mod.add_constr(
            xsum(X[i][j] for i in PalletsRange) == 1, name=f"kept_{j}"
        )
    for i in PalletsRange:
        # each pallet may receive at most one consolidated
        mod.add_constr(
            xsum(X[i][j] for j in KeptRange) <= 1, name=f"pallet_{i}"
        )                 

    status = mod.optimize(max_seconds=10) 

    print(status)

    if mod.num_solutions:

        tTotAccum = 0
        for i, _ in enumerate(pallets):

            tTotAccum += float(140) * pallets[i].D

            for j in KeptRange:

                if X[i][j].x >= 0.99:

                    tTotAccum += float(kept[j].W) * pallets[i].D    

                    # pallets[i].To = kept[j].To # define pallet destination
                    pallets[i].Dests[k] = kept[j].To
                    kept[j].P = i # put the consolidated in the best position to minimize torque                        

        tTotAccum /= maxTorque
        print("\n----- OptCGCons relative torque: %.2f -----" % tTotAccum)



if __name__ == "__main__":

    # two_up = path.abspath(path.join(__file__, "../..")) + "/data"

    # FOR TESTING ONLY
    # DO NOT CHANGE THESE PARAMETERS
    scenario = 4
    instance = 1
    node     = 1
    tix      = 0 # tour index in the permutations
    tour     = [0, 1, 2, 3, 4, 0]

    cfg = methods.Config(scenario)

    pallets = methods.loadPallets(cfg.Acft)
    PalletsRange = range(len(pallets))

    nix        = tour.index(node) # node index in the tour
    unattended = tour[nix+1:]

    items = methods.loadNodeItems(scenario, instance, node, unattended, "")

    # pallets capacity
    weiCap = 0
    volCap = 0
    for p in pallets:
        weiCap += p.W
        volCap += p.V

    # smaller aircrafts may have a payload lower than pallets capacity
    if weiCap > cfg.Payload:
        weiCap = cfg.Payload

    # consolidated generated in node 0, to be dealt with in node 1
    cons = []
    id = len(items)
    # cons.append(       Item(id,   -1,    w,    s,   v,  k, to) )
    cons.append( methods.Item(id+0, -1, 1897, 1387, 7.0,  0, 1) )
    cons.append( methods.Item(id+1, -1, 1962, 1221, 7.0,  0, 2) )
    cons.append( methods.Item(id+2, -1, 2613, 1486, 10.0, 0, 3) )
    cons.append( methods.Item(id+3, -1, 2666, 1650, 10.0, 0, 4) )
    cons.append( methods.Item(id+4, -1, 4153, 2672, 14.8, 0, 1) )
    cons.append( methods.Item(id+5, -1, 4053, 2085, 14.8, 0, 2) )
    cons.append( methods.Item(id+6, -1, 3562, 1967, 14.8, 0, 3) )
    cons.append( methods.Item(id+7, -1, 3926, 2405, 14.8, 0, 4) )
    cons.append( methods.Item(id+8, -1, 3593, 2606, 14.8, 0, 1) )
    cons.append( methods.Item(id+9, -1, 4145, 2440, 14.8, 0, 2) )
    cons.append( methods.Item(id+10, -1, 3912, 1990, 14.8, 0, 3) )
    cons.append( methods.Item(id+11, -1, 3774, 2136, 14.8, 0, 4) )
    cons.append( methods.Item(id+12, -1, 3745, 1975, 14.8, 0, 1) )
    cons.append( methods.Item(id+13, -1, 3670, 1740, 14.8, 0, 2) )
    cons.append( methods.Item(id+14, -1, 3410, 1937, 14.8, 0, 3) )
    cons.append( methods.Item(id+15, -1, 3542, 2381, 14.8, 0, 4) )
    cons.append( methods.Item(id+16, -1, 3306, 2248, 14.8, 0, 1) )
    cons.append( methods.Item(id+17, -1, 3385, 1985, 14.8, 0, 2) )

    # consolidated contents not destined to this point are kept on board
    kept = []
    id = len(items)
    for c in cons:
        if c.To in unattended:
            c.ID = id
            c.K = node # now the consolidated from the previous node depart from this node
            kept.append(c)
            id += 1

    start_time = time()

    OptCGCons(kept, pallets, cfg.aircraft.maxTorque, "GRB")

    elapsed = time() - start_time
    elapsed = math.ceil(1000*elapsed)  # milliseconds    

    print(f"elapsed: {elapsed} ms ")