
# include the MILP Solver: https://www.python-mip.com/
from mip import Model, xsum, maximize, BINARY
from time import time
    
def Solve( pallets, items, cfg, k, secBreak, dictItems ):
# def Solve(method, pallets, items, cons, nodes, maxTorque, Limited, maxWei, secBreak):

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

    # decision matrix for which items will be put in which pallet in which node
    X = [
            [ mod.add_var(name=f"X({i},{j})", var_type=BINARY) for j in set_N
            ]                                                  for i in set_M
        ]
         
    # consolidated are included in items
    score = xsum( X[i][j] * items[j].S  for i in set_M for j in set_N)

    mod.objective = maximize( score )

    # CONSTRAINTS ----------------------------------------------------------------------------
    xsum(X[i][j] for i in set_M for j in set_N) <= 1  # each must be included at most once

    for i in set_M:

        for j in set_N:

            # items must be grouped by the same destination
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

    # the final torque must be between minus maxTorque and maxTorque

        palletWeights[i] = 140 + xsum(X[i][j] * items[j].W  for j in set_N) 

    sumTorques = xsum( pallets[i].D * palletWeights[i] for i in set_M )
    mod.add_constr(
        sumTorques <=    cfg.maxTorque
    )
    mod.add_constr(
        sumTorques >= -1*cfg.maxTorque
    )

    # the aircraft payload (or maximum pallets capacities) must not be exceeded
    sumWeights = 140*M + xsum(X[i][j] * items[j].W  for i in set_M for j in set_N) 
    mod.add_constr(
        sumWeights <= cfg.weiCap
    )    

    # lateral torque was never significant. So, we did not include lateral torque constraints

    status = mod.optimize()        

    print(status)

    # checking if a solution was found
    if mod.num_solutions:
        for j in set_N:
            for i in set_M:
                if X[i][j].x >= 0.99: # put items in solution

                    dictItems["solItems"][j] = i # item "j" is allocated to pallet "i"


if __name__ == "__main__":

    print("----- Please execute module main -----")
