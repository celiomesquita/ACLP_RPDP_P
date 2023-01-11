from mip import Model, xsum, minimize, BINARY, CBC, OptimizationStatus, GRB

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
            for j in KeptRange:
                if X[i][j].x >= 0.99:
                    tTotAccum += float(kept[j].W) * pallets[i].D
                    pallets[i].Dests[k] = kept[j].To
                    kept[j].P = i # put the consolidated in the best position to minimize torque                        

        tTotAccum /= maxTorque
        print("\n----- OptCGCons relative torque: %.2f -----" % tTotAccum)


if __name__ == "__main__":

    print("----- Please execute module main -----")
   