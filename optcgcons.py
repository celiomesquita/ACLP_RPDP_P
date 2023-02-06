import gurobipy as gp
from gurobipy import GRB

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

    tTotAccum = 0
    for i, _ in enumerate(pallets):
        pallets[i].Dest[k] = -1 # reset pallets destinations from this node
        for j in KeptRange:
            if X[i][j].x >= 0.99:
                tTotAccum += float(kept[j].W) * pallets[i].D
                pallets[i].Dest[k] = kept[j].To
                kept[j].P = i # put the consolidated in the best position to minimize torque                        

    msgdict = {2:'Optimal', 3:'Infeasible', 13:"Suboptimal", 9:"Time limited"}
    print(f"--- Center of gravity deviation minimized as {msgdict[mod.status]} {tTotAccum/maxTorque:.2f}")

    return tTotAccum

if __name__ == "__main__":

    print("----- Please execute module main -----")
   