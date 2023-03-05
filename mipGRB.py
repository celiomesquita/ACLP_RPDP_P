
# include the MILP Solver: https://www.python-mip.com/
# from mip import Model, xsum, maximize, BINARY

import gurobipy as gp
from gurobipy import GRB

# from time import time
# import multiprocessing as mp
# import numpy as np

def Solve( pallets, items, cfg, k, secBreak, nodeTorque, solDict, itemsDict):

    # itemsDict to control items inclusion feasibility

    score = 0
    for p in pallets:
        score += p.PCS

    N = len(items)
    M = len(pallets)

    palletWeights = [0 for _ in pallets]

    set_M = range( M ) # i, pallets
    set_N = range( N ) # j, items

    # initialize a model
    mod = gp.Model()
    mod = mod.relax()
    
    # decision matrix for which items will be put in which pallet in node "k"         
    # X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.BINARY) for j in set_N ] for i in set_M ]  
    X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.CONTINUOUS) for j in set_N ] for i in set_M ]   

    mod.setObjective(sum( X[i][j] * items[j].S for i in set_M for j in set_N ))

    mod.ModelSense = GRB.MAXIMIZE

    # CONSTRAINTS ----------------------------------------------------------------------------
    
    for j in set_N:
        # each item must be included at most once
        mod.addConstr(
            sum(X[i][j] for i in set_M ) <= 1
        )

    for i in set_M:

        # items must be grouped in a pallet with the same destination
        for j in set_N:
            mod.addConstr(
                X[i][j] <= X[i][j] * ( pallets[i].Dest[k] - items[j].To + 1 )
            )
            mod.addConstr(
                X[i][j] <= X[i][j] * ( items[j].To - pallets[i].Dest[k] + 1 )
            )

        # Pallet weight constraint                      PCW: pallet current weight
        mod.addConstr(
            sum(X[i][j] * items[j].W for j in set_N) + pallets[i].PCW <= pallets[i].W
        )

        # Pallet volume constraint
        mod.addConstr(
            sum(X[i][j] * items[j].V for j in set_N) + pallets[i].PCV <= pallets[i].V
        )

        # for torque calculation
        palletWeights[i] = 140 + sum(X[i][j] * items[j].W  for j in set_N) + pallets[i].PCW

    # the final torque must be between minus maxTorque and maxTorqu

    sumTorques = sum(pallets[i].D * palletWeights[i] for i in set_M)
    mod.addConstr(
        sumTorques <=    cfg.maxTorque
    )
    mod.addConstr(
        sumTorques >= -1*cfg.maxTorque
    )    

    # the aircraft payload (or maximum pallets capacities) must not be exceeded 
    mod.addConstr(
        sum(palletWeights[i] for i in set_M) <= cfg.weiCap
    )  
    # lateral torque was never significant. So, we did not include lateral torque constraints

    # msgdict = {2:'Optimal', 3:'Infeasible', 13:"Suboptimal", 9:"Time limited"}

    mod.setParam('OutputFlag', 0)
    mod.Params.TimeLimit = secBreak
    mod.Params.Threads = 1

    # mod.setParam(GRB.Param.LogToConsole, 0)

    mod.optimize()

    bound = 0
    for i in set_M:
        bound += pallets[i].PCS

    # checking if a solution was found
    if mod.SolCount > 0:

        bound = mod.ObjBound

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
