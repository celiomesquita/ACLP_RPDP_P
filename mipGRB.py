
# include the MILP Solver: https://www.python-mip.com/
from mip import Model, xsum, maximize, BINARY
from time import time
import common
import multiprocessing as mp
import numpy as np

    
def Solve( pallets, items, cfg, k, secBreak, solTorque, solDict, mpItemsDict ):
    
    # mpItemsDict to control items inclusion feasibility

    N = len(items)
    M = len(pallets)

    palletWeights = [0 for _ in pallets]

    set_M = range( M ) # i, pallets
    set_N = range( N ) # j, items

    # initialize a model
    mod = Model()
    mod.solver_name = "GRB"
    mod.threads     = 1
    mod.max_seconds = secBreak

    # decision matrix for which items will be put in which pallet in node "k"
    X = [
            [ mod.add_var(name=f"X({i},{j})", var_type=BINARY) for j in set_N
            ]                                                  for i in set_M
        ]
         
    # consolidated are included in items
    scoreItem = xsum( X[i][j] * items[j].S     for i in set_M for j in set_N if items[j].P == -1)
    scoreCons = xsum( X[i][j] * items[j].S*50  for i in set_M for j in set_N if items[j].P == -2)

    mod.objective = maximize( scoreItem + scoreCons )

    # CONSTRAINTS ----------------------------------------------------------------------------
    
    for j in set_N:
        # each item must be included at most once
        mod.add_constr(
            xsum(X[i][j] for i in set_M ) <= 1
        )

        # each consolidated must be included exactly once: deactivated because some infesibilities
        # if items[j].P == -2: # pallet not set
        #     mod.add_constr(
        #         xsum(X[i][j] for i in set_M ) == 1
        #     )

    for i in set_M:

        # items must be grouped in a pallet with the same destination
        for j in set_N:
            mod.add_constr(
                X[i][j] <= X[i][j] * ( pallets[i].Dests[k] - items[j].To + 1 )
            )
            mod.add_constr(
                X[i][j] <= X[i][j] * ( items[j].To - pallets[i].Dests[k] + 1 )
            )

        # Pallet weight constraint
        mod.add_constr(
            xsum(X[i][j] * items[j].W for j in set_N) <= pallets[i].W
        )

        # Pallet volume constraint
        mod.add_constr(
            xsum(X[i][j] * items[j].V for j in set_N) <= pallets[i].V
        )

        # for torque calculation
        palletWeights[i] = 140 + xsum(X[i][j] * items[j].W  for j in set_N)

    # the final torque must be between minus maxTorque and maxTorque
    sumTorques = xsum(pallets[i].D * palletWeights[i] for i in set_M)
    mod.add_constr(
        sumTorques <=    cfg.maxTorque
    )
    mod.add_constr(
        sumTorques >= -1*cfg.maxTorque
    )

    # the aircraft payload (or maximum pallets capacities) must not be exceeded
    mod.add_constr(
        xsum(palletWeights[i] for i in set_M) <= cfg.weiCap
    )    

    # lateral torque was never significant. So, we did not include lateral torque constraints

    status = mod.optimize()        

    print(status)
    # print(mod.objective_value)

    # checking if a solution was found
    if mod.num_solutions:   

        # reset empty pallets torque
        solTorque.value = 0.0
        for i in set_M:
            solTorque.value += 140.0 * pallets[i].D         

            for j in set_N:

                solDict["solMatrix"][N*i+j] = 0

                if X[i][j].x >= 0.99: # put items in solution

                    solDict["solMatrix"][N*i+j] = 1

                    solTorque.value += items[j].W * pallets[i].D

                    if mpItemsDict["mpItems"][j] == 0:
                        mpItemsDict["mpItems"][j] = 1

if __name__ == "__main__":

    print("----- Please execute module main -----")
