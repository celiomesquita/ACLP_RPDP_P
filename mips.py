
# include the MILP Solver: https://www.python-mip.com/
from mip import Model, xsum, maximize, BINARY, CBC, OptimizationStatus, GRB

import math
from os import path
from time import time
import methods

def readShimsValue(scenario, instance, pi, node):

    dirname = f"./Shims/{scenario}"

    fname = f"{dirname}/{instance}_{pi}_{node}.txt"

    reader = open(fname, "r")
    lines = reader.readlines() 

    value = 0

    try:
        for line in lines:
            cols  = line.split()
            value = int(cols[0])

    finally:
        reader.close()    

    return value

# these items have also consolidated
def Solve(method, pallets, items, pi, node, cfg, Limited, k):

    itemsRange   = range(len(items))
    numPallets = len(pallets)
    palletsRange = range(numPallets)

    # initialize a model
    mod = Model(solver_name=method)
    mod.threads=1
    if Limited:
        mod.max_seconds = methods.SEC_BREAK

    # decision matrix for which items will be put in which pallet
    X = [ [ mod.add_var(name=f"X({i},{j})", var_type=BINARY)
        for j in itemsRange ]
        for i in palletsRange ]        

    # define the objective function value: equation 2
    #scoreIt   = xsum( X[i][j] * items[j].S   for i in palletsRange for j in itemsRange )

    scoreIt   = xsum( X[i][j] * items[j].S   for i in palletsRange for j in itemsRange if items[j].P==-1 )
    scoreCons = xsum( X[i][j] * items[j].S*2 for i in palletsRange for j in itemsRange if items[j].P==-2 )

    # items[j].P is always -2 for consolidated because "mips" does not use OptCgCons

    mod.objective = maximize( scoreIt + scoreCons )  
    #mod.objective = maximize( scoreIt )      

    # CONSTRAINTS ----------------------------------------------------------------------------

    for j in itemsRange:

        # each item must be included at most once
        mod.add_constr(
            xsum(X[i][j] for i in palletsRange) <= 1, name=f"eq19_{j}"
        )

        # each consolidated must be included exactly once:dectivated because of some nodes infeasibilities
        #if items[j].P == -2:
        #    mod.add_constr(
        #        xsum(X[i][j] for i in palletsRange ) == 1, name=f"eq20_{j}"
        #    )

    # for each pallet
    for i in palletsRange:

        for j in itemsRange:

            # items must be grouped by the same destination
            mod.add_constr(
                X[i][j] <= X[i][j] * ( pallets[i].Dests[k] - items[j].To + 1 ), name=f"eq21_{i}{j}_a"
            )
            mod.add_constr(
                X[i][j] <= X[i][j] * ( items[j].To - pallets[i].Dests[k] + 1 ), name=f"eq21_{i}{j}_b"
            )

        # Pallet weight constraint
        mod.add_constr(
            xsum(X[i][j] * items[j].W for j in itemsRange) <= pallets[i].W, name =f"eq17_{i}"
        )

        # Pallet volume constraint
        mod.add_constr(
            xsum(X[i][j] * items[j].V for j in itemsRange) <= pallets[i].V, name =f"eq18_{i}"
        )

    # the final torque must be between minus maxTorque and maxTorque
    sumTorques = xsum( pallets[i].D * ( 140 + xsum(X[i][j] * items[j].W  for j in itemsRange)  ) for i in palletsRange )
    mod.add_constr(
        sumTorques <=    cfg.aircraft.maxTorque, name="eq15_aft"
    )
    mod.add_constr(
        sumTorques >= -1*cfg.aircraft.maxTorque, name="eq15_fwd"
    )    

    # the aircraft payload (or maximum pallets capacities) must not be exceeded
    sumWeights = 140*numPallets + xsum(X[i][j] * items[j].W  for j in itemsRange) 
    mod.add_constr(
        sumWeights <= cfg.weiCap, name =f"eq16"
    )    

    # lateral torque was never significant. So, we did not include lateral torque constraints

    print("\n-----TOUR %d %s BEGIN-----------------------------------------------------------" %
            (pi, methods.CITIES[node]))

    status = -1
    status = mod.optimize()        

    print(status)

    if status == OptimizationStatus.INFEASIBLE:
        return [], -1  # empty solution

    opt = 0
    if status == OptimizationStatus.OPTIMAL:
        opt = 1

    # checking if a solution was found
    if mod.num_solutions:

        E = []

        for i in palletsRange:

            row = []
            for j in itemsRange:

                row.append(0)

                if X[i][j].x >= 0.99: # put items in solution

                    row[j] = 1

            E.append(row)

        return E, opt
    
    return [], -1  # empty solution

    
if __name__ == "__main__":

    print("----- Please execute module main_test -----")
