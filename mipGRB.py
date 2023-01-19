
# include the MILP Solver: https://www.python-mip.com/
from mip import Model, xsum, maximize, BINARY
from time import time
import common
import multiprocessing as mp
import numpy as np

    
def Solve( pallets, items, cfg, k, secBreak, solTorque, dictItems ):

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
    score = xsum( X[i][j] * items[j].S  for i in set_M for j in set_N)

    mod.objective = maximize( score )

    # CONSTRAINTS ----------------------------------------------------------------------------
    
    # each item must be included at most once
    xsum(X[i][j] for j in set_N for i in set_M ) <= 1

    # each consolidated must be included exactly once
    xsum(X[i][j] for j in set_N for i in set_M if dictItems["solItems"][j] == i) == 1

    for i in set_M:

        for j in set_N:

            # items must be grouped in a pallet with the same destination
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

        sNodeAccum = 0.
        wNodeAccum = 0.
        vNodeAccum = 0.
        sol = ""
        pAccum   = [0.0 for _ in pallets]

        solItems = mp.Array('i', [0]*N*M)
   
        # reset empty pallets torque
        solTorque.value = 0.0
        for i in set_M:
            solTorque.value += 140.0 * pallets[i].D         

            for j in set_N:

                if X[i][j].x >= 0.99: # put items in solution

                    dictItems["solItems"][j] = i # item "j" is allocated to pallet "i"
                    solItems[N*i+j]          = 1

                    solTorque.value += items[j].W * pallets[i].D 

                    sNodeAccum += float(items[j].S)
                    wNodeAccum += float(items[j].W)
                    vNodeAccum += float(items[j].V)

                    pAccum[i] += float(items[j].W)

            print(f"1 pAccum[{i}]: {pAccum[i]}")

        epsilom = solTorque.value/cfg.maxTorque

        vol = vNodeAccum/cfg.volCap
        wei = wNodeAccum/cfg.weiCap

        sol += f"Score: {sNodeAccum:.0f}\t"
        sol += f"Weight: {wei:.2f}\t"
        sol += f"Volume: {vol:.2f}\t"
        sol += f"Torque: {epsilom:.2f}\n"

        print(f"1: mipGRB X[i][j].x solution ----- \n{sol}")

        sNodeAccum = 0.
        wNodeAccum = 0.
        vNodeAccum = 0.
        sol = ""
        # reset empty pallets torque
        torque = 0.0
        for i in set_M:
            torque += 140.0 * pallets[i].D
            pAccum[i] = 0.0      

        Y = np.reshape(solItems, (-1, N))

        for i, row in enumerate(Y):
            for j, content in enumerate(row):

                if content == 1:

                    torque += items[j].W * pallets[i].D

                    sNodeAccum += float(items[j].S)
                    wNodeAccum += float(items[j].W)
                    vNodeAccum += float(items[j].V)

                    pAccum[i] += float(items[j].W)


        for i in set_M:
            print(f"2 pAccum[{i}]: {pAccum[i]}")

        epsilom = torque/cfg.maxTorque

        vol = vNodeAccum/cfg.volCap
        wei = wNodeAccum/cfg.weiCap

        sol += f"Score: {sNodeAccum:.0f}\t"
        sol += f"Weight: {wei:.2f}\t"
        sol += f"Volume: {vol:.2f}\t"
        sol += f"Torque: {epsilom:.2f}\n"

        print(f"2: mipGRB solItems solution ----- \n{sol}")



if __name__ == "__main__":

    print("----- Please execute module main -----")
