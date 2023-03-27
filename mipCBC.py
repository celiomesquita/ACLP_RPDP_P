
# include the MILP Solver: https://www.python-mip.com/
from mip import Model, xsum, maximize, BINARY, CBC, CONTINUOUS

def Solve( pallets, items, cfg, k, secBreak, nodeTorque, solDict, itemsDict):

    # itemsDict to control items inclusion feasibility

    score = 0
    for p in pallets:
        score += p.PCS

    N = len(items)
    M = len(pallets)


    set_M = range( M ) # i, pallets
    set_N = range( N ) # j, items

    mod = Model(solver_name=CBC)
    mod.verbose = 0 # hide messages
    
    # decision matrix for which items will be put in which pallet in node "k"         
    X = [ [ mod.add_var(name=f"X[{i}],[{j}]", var_type=CONTINUOUS) for j in set_N ] for i in set_M ]   

    mod.objective = maximize( xsum( X[i][j] * items[j].S for i in set_M for j in set_N ) )

    # CONSTRAINTS ----------------------------------------------------------------------------
    
    for j in set_N:
        # each item must be included at most once
        mod.add_constr(
            sum(X[i][j] for i in set_M ) <= 1
        )

    palletWeights = [0 for _ in pallets]
    itemsWeights  = [0 for _ in pallets]
    itemsVolumes  = [0 for _ in pallets]

    for i in set_M:

        # items must be grouped in a pallet with the same destination
        for j in set_N:
            mod.add_constr(
                X[i][j] <= X[i][j] * ( pallets[i].Dest[k] - items[j].To + 1 )
            )
            mod.add_constr(
                X[i][j] <= X[i][j] * ( items[j].To - pallets[i].Dest[k] + 1 )
            )

        # Pallet weight constraint                      PCW: pallet current weight
        itemsWeights[i] = xsum(X[i][j] * items[j].W  for j in set_N)
        # mod.addConstr(
        #     itemsWeights[i] + pallets[i].PCW <= pallets[i].W
        # )

        # Pallet volume constraint
        itemsVolumes[i] = xsum(X[i][j] * items[j].V  for j in set_N)
        mod.add_constr(
            itemsVolumes[i] + pallets[i].PCV <= pallets[i].V
        )

        # for torque calculation
        palletWeights[i] = pallets[i].PCW + 140 + itemsWeights[i]

    # the final torque must be between minus maxTorque and maxTorqu

    sumTorques = xsum(pallets[i].D * palletWeights[i] for i in set_M)
    mod.add_constr(
        sumTorques <=    cfg.maxTorque
    )
    mod.add_constr(
        sumTorques >= -1*cfg.maxTorque
    )    

    # lateral torque was never significant. So, we did not include lateral torque constraints

    mod.optimize()

    bound = 0
    for i in set_M:
        bound += pallets[i].PCS

    # checking if a solution was found
    if mod.num_solutions:

        bound = mod.objective_bound

        # reset empty pallets torque
        nodeTorque.value = 0.0
        for i in set_M:
            nodeTorque.value += 140.0 * pallets[i].D         

            for j in set_N:

                solDict["solMatrix"][N*i+j] = 0

                if X[i][j].x >= 0.99: # put items in solution

                    solDict["solMatrix"][N*i+j] = 1

                    nodeTorque.value += items[j].W * pallets[i].D

                    itemsDict["mpItems"][j] = 1

                    # consolidated are yet on pallets
                    pallets[i].PCW += items[j].W
                    pallets[i].PCS += items[j].S
                    pallets[i].PCV += items[j].V

    return mod.status, bound

if __name__ == "__main__":

    print("----- Please execute module main -----")
