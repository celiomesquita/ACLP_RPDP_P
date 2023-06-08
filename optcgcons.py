# import gurobipy as gp
# from gurobipy import GRB

from mip import Model, xsum, minimize, BINARY, CBC

import common

# optimize consolidated positions to minimize CG deviation
def OptCGCons(kept, pallets, k, nodeTorque):

    KeptRange    = range(len(kept))
    PalletsRange = range(len(pallets))

    # mod = gp.Model()
    mod = Model(solver_name=CBC)
    # mod.setParam('OutputFlag', 0)
    mod.verbose = 0 # hide messages

    # X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.BINARY) for j in KeptRange ] for i in PalletsRange ]  
    X = [ [ mod.add_var(name=f"X[{i}],[{j}]", var_type=BINARY) for j in KeptRange ] for i in PalletsRange ]     

    torque1 = xsum( X[i][j] * ((140+kept[j].W) * pallets[i].D) for i in PalletsRange for j in KeptRange if pallets[i].D > 0 ) 
    torque2 = xsum( X[i][j] * ((140+kept[j].W) * pallets[i].D) for i in PalletsRange for j in KeptRange if pallets[i].D < 0 )

    # mod.setObjective(torque1-torque2)
    # mod.ModelSense = GRB.MINIMIZE
    mod.objective = minimize( torque1-torque2 )

    for j in KeptRange:
        mod.add_constr(
            xsum(X[i][j] for i in PalletsRange) == 1
        )        
    for i in PalletsRange:                
        mod.add_constr(
            xsum(X[i][j] for j in KeptRange) <= 1
        ) 

    mod.optimize() 

    nodeTorque.value = 0
    for i, _ in enumerate(pallets):
        pallets[i].Dest[k] = -1 # reset pallets destinations from this node
        nodeTorque.value += 140.0 * pallets[i].D
        for j in KeptRange:
            if X[i][j].x >= 0.99:
                nodeTorque.value += float(kept[j].W) * pallets[i].D
                pallets[i].Dest[k] = kept[j].To
                kept[j].P = i # put the consolidated in the best position to minimize torque                        


# After solved, optimize consolidated positions to minimize CG deviation
def minCGdev(pallets, k, nodeTorque, cfg):

    cons = [common.Item(p.ID, -2, p.PCW, p.PCS, p.PCV, k, p.Dest[k]) for p in pallets]

    ConsRange    = range(len(cons))
    PalletsRange = range(len(pallets))

    for i in PalletsRange:
        pallets[i].reset(cfg.numNodes)

    # mod = gp.Model()
    mod = Model(solver_name=CBC)
    # mod.setParam('OutputFlag', 0)
    mod.verbose = 0 # hide messages

    Y = [ [ mod.add_var(name=f"X[{i}],[{q}]", var_type=BINARY) for q in ConsRange ] for i in PalletsRange ] 

    torque1 = xsum( Y[i][q] * ((140+cons[q].W) * pallets[i].D) for i in PalletsRange for q in ConsRange if pallets[i].D > 0 ) 
    torque2 = xsum( Y[i][q] * ((140+cons[q].W) * pallets[i].D) for i in PalletsRange for q in ConsRange if pallets[i].D < 0 )

    mod.objective = minimize( torque1-torque2 )

    for q in ConsRange:
        mod.add_constr(
            xsum(Y[i][q] for i in PalletsRange) <= 1
        )        
    for i in PalletsRange:                
        mod.add_constr(
            xsum(Y[i][q] for q in ConsRange) == 1
        )

    mod.optimize() 

    nodeTorque.value = 0

    for i, _ in enumerate(pallets):

        for q in ConsRange:
            if Y[i][q].x >= 0.99:
                cons[q].P = i # put the consolidated in the best position to minimize torque                        
                pallets[i].Dest[k] = cons[q].To                
                pallets[i].PCW     = cons[q].W
                pallets[i].PCV     = cons[q].V
                pallets[i].PCS     = cons[q].S

                nodeTorque.value += (140 + cons[q].W) * pallets[i].D

# After solved, minimize the sum of ramp door distances for the next node pallets
def minRampDist(pallets, k, tour, rampDistCG, cfg, nodeTorque):


    node = tour.nodes[k]
    next = tour.nodes[k+1]

    cons = [common.Item(p.ID, -2, p.PCW, p.PCS, p.PCV, node, p.Dest[k]) for p in pallets]

    ConsRange    = range(len(cons))
    PalletsRange = range(len(pallets))

    # mod = gp.Model()
    mod = Model(solver_name=CBC)
    # mod.setParam('OutputFlag', 0)
    mod.verbose = 0 # hide messages

    # X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.BINARY) for j in ConsRange ] for i in PalletsRange ] 
    X = [ [ mod.add_var(name=f"X[{i}],[{j}]", var_type=BINARY) for j in ConsRange ] for i in PalletsRange ]    

    rampDist = xsum( X[i][j] * (rampDistCG - pallets[i].D) for i in PalletsRange for j in ConsRange if cons[j].To == next ) 
    
    # mod.setObjective(rampDist)
    # mod.ModelSense = GRB.MINIMIZE
    mod.objective = minimize(rampDist)

    for j in ConsRange:
        mod.add_constr(
            xsum(X[i][j] for i in PalletsRange) == 1
        )

    for i in PalletsRange:                
        mod.add_constr(
            xsum(X[i][j] for j in ConsRange) == 1
        )

    sumTorques = xsum(pallets[i].D * ( 140 + cons[j].W ) for i in PalletsRange for j in ConsRange)

    mod.add_constr(
        sumTorques <=    cfg.maxTorque
    )
    mod.add_constr(
        sumTorques >= -1*cfg.maxTorque
    )  

    mod.optimize()

    # if mod.status == 3: # infeasible
    #     print(f"Dist from ramp door model infeasible")
    # else:
    #     print(f"mod.ObjVal: {mod.ObjVal}")

    # if mod.SolCount > 0:
    if mod.num_solutions:

        nodeTorque.value = 0

        for i in PalletsRange:

            for j in ConsRange:

                if X[i][j].x >= 0.99:

                    cons[j].P = i
                    pallets[i].Dest[k] = cons[j].To                
                    pallets[i].PCW     = cons[j].W
                    pallets[i].PCV     = cons[j].V
                    pallets[i].PCS     = cons[j].S

                    nodeTorque.value += (140 + cons[j].W) * pallets[i].D                     


if __name__ == "__main__":

    print("----- Please execute module main -----")
   