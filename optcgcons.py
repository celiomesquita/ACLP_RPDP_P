import gurobipy as gp
from gurobipy import GRB
import common

# optimize consolidated positions to minimize CG deviation
def OptCGCons(kept, pallets, maxTorque, k):

    KeptRange    = range(len(kept))
    PalletsRange = range(len(pallets))

    mod = gp.Model()
    mod.setParam('OutputFlag', 0)

    X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.BINARY) for j in KeptRange ] for i in PalletsRange ]      

    torque1 = sum( X[i][j] * (kept[j].W *    pallets[i].D) for i in PalletsRange for j in KeptRange ) 
    torque2 = sum( X[i][j] * (kept[j].W * -1*pallets[i].D) for i in PalletsRange for j in KeptRange ) 
    mod.setObjective(torque1 + torque2)

    mod.ModelSense = GRB.MINIMIZE

    for j in KeptRange:
        mod.addConstr(
            sum(X[i][j] for i in PalletsRange) == 1
        )        
    for i in PalletsRange:                
        mod.addConstr(
            sum(X[i][j] for j in KeptRange) <= 1
        ) 

    mod.optimize() 

    torque = 0
    for i, _ in enumerate(pallets):
        pallets[i].Dest[k] = -1 # reset pallets destinations from this node
        torque += 140.0 * pallets[i].D
        for j in KeptRange:
            if X[i][j].x >= 0.99:
                torque += float(kept[j].W) * pallets[i].D
                pallets[i].Dest[k] = kept[j].To
                kept[j].P = i # put the consolidated in the best position to minimize torque                        

    print(f"--- {common.CITIES[k]} CG deviation minimized {torque/maxTorque:.2f}")

    return torque

# After solved, optimize consolidated positions to minimize CG deviation
def OptCG(pallets, k, nodeTorque):

    cons = [common.Item(p.ID, -2, p.PCW, p.PCS, p.PCV, k, p.Dest[k]) for p in pallets]

    ConsRange    = range(len(cons))
    PalletsRange = range(len(pallets))

    mod = gp.Model()
    mod.setParam('OutputFlag', 0)

    X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.BINARY) for j in ConsRange ] for i in PalletsRange ]      

    torque1 = sum( X[i][j] * ((140+cons[j].W) *    pallets[i].D) for i in PalletsRange for j in ConsRange ) 
    torque2 = sum( X[i][j] * ((140+cons[j].W) * -1*pallets[i].D) for i in PalletsRange for j in ConsRange ) 
    mod.setObjective(torque1 + torque2)

    mod.ModelSense = GRB.MINIMIZE

    for j in ConsRange:
        mod.addConstr(
            sum(X[i][j] for i in PalletsRange) == 1
        )        
    for i in PalletsRange:                
        mod.addConstr(
            sum(X[i][j] for j in ConsRange) == 1
        ) 

    mod.optimize() 

    nodeTorque.value = mod.objVal

    for i, _ in enumerate(pallets):

        for j in ConsRange:
            if X[i][j].x >= 0.99:
                pallets[i].Dest[k] = cons[j].To
                cons[j].P = i # put the consolidated in the best position to minimize torque                        


# After solved, minimize the sum of ramp door distances for the next node pallets
def OptRampDist(pallets, k, tour, rampDistCG, cfg, nodeTorque):

    palletWeights = [0 for _ in pallets]

    node = tour.nodes[k]
    next = tour.nodes[k+1]

    cons = [common.Item(p.ID, -2, p.PCW, p.PCS, p.PCV, node, p.Dest[k]) for p in pallets]

    ConsRange    = range(len(cons))
    PalletsRange = range(len(pallets))

    for j in ConsRange:
        palletWeights[j] = 140 + cons[j].W 

    mod = gp.Model()
    mod.setParam('OutputFlag', 0)

    X = [ [ mod.addVar(name=f"X[{i}],[{j}]", vtype=GRB.BINARY) for j in ConsRange ] for i in PalletsRange ]      

    rampDist = sum( X[i][j] * (rampDistCG - pallets[i].D) for i in PalletsRange for j in ConsRange if cons[j].To == next ) 
    
    mod.setObjective(rampDist)

    mod.ModelSense = GRB.MINIMIZE

    for j in ConsRange:
        mod.addConstr(
            sum(X[i][j] for i in PalletsRange) == 1
        )
    for i in PalletsRange:                
        mod.addConstr(
            sum(X[i][j] for j in ConsRange) == 1
        )

    sumTorques = sum(pallets[i].D * palletWeights[i] for i in PalletsRange)
    mod.addConstr(
        sumTorques <=    cfg.maxTorque
    )
    mod.addConstr(
        sumTorques >= -1*cfg.maxTorque
    )  

    mod.optimize()

    if mod.status == 3: # not infeasible
        print(f"Dist from ramp door model infeasible")
    else:
        print(f"mod.objVal: {mod.objVal}")

    if mod.SolCount > 0:
        nodeTorque.value = 0
        for i in PalletsRange:
            for j in ConsRange:
                if X[i][j].x >= 0.99:
                    pallets[i].Dest[k] = cons[j].To
                    cons[j].P = i
                    nodeTorque.value += (140 + cons[j].W) * pallets[i].D                 

if __name__ == "__main__":

    print("----- Please execute module main -----")
   