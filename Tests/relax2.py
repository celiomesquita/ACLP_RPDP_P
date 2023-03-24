#!/usr/bin/env python

# This is the GAP per Wolsey, pg 182, using Lagrangian Relaxation

from gurobipy import GRB, Model

model = Model('GAP per Wolsey with Lagrangian Relaxation')
model.modelSense = GRB.MAXIMIZE
model.setParam('OutputFlag', False) # turns off solver chatter

b = [15, 15, 15]
c = [
    [ 6, 10,  1],
    [12, 12,  5],
    [15,  4,  3],
    [10,  3,  9],
    [ 8,  9,  5]
]
a = [
    [ 5,  7,  2],
    [14,  8,  7],
    [10,  6, 12],
    [ 8,  4, 15],
    [ 6, 12,  5]
]

# x[i][j] = 1 if i is assigned to j
x = []
for i in range(len(c)):
    x_i = []
    for j in c[i]:
        x_i.append(model.addVar(vtype=GRB.BINARY))
    x.append(x_i)

# As stated, the GAP has these following constraints. We dualize these into
# penalties instead, using variables so we can easily extract their values.
penalties = [model.addVar() for _ in x]
model.update()

# Dualized constraints: sum j: x_ij <= 1 for all i
for p, x_i in zip(penalties, x):
    model.addConstr(p == 1 - sum(x_i))

# sum i: a_ij * x_ij <= b[j] for all j
for j in range(len(b)):
    model.addConstr(sum(a[i][j] * x[i][j] for i in range(len(x))) <= b[j])

# u[i] = Lagrangian Multiplier for the set packing contraint i
u = [2.0] * len(x)

# Re-optimize until either we have run a certain number of iterations
# or complementary slackness conditions apply.
for k in range(1, 101):
    # max sum i,j: c_ij * x_ij
    model.setObjective(
        sum(
            # Original objective function
            sum(c_ij * x_ij for c_ij, x_ij in zip(c_i, x_i))
            for c_i, x_i in zip(c, x)
        ) + sum (
            # Penalties for dualized constraints
            u_j * p_j for u_j, p_j in zip(u, penalties)
        )
    )
    model.optimize()

    print (f'iteration {k} obj={model.objVal:.1f} u={u} penalties={[p.x for p in penalties]}') 

    # Test for complementary slackness
    stop = True
    eps = 10e-6
    for u_i, p_i in zip(u, penalties):
        if abs(u_i) > eps and abs(p_i.x) > eps:
            stop = False
            break

    if stop:
        print ('primal feasible & optimal')
        break

    else:
        s = 1.0 / k
        for i in range(len(x)):
            u[i] = max(u[i] - s*(penalties[i].x), 0.0)

# Pull objective and variable values out of model
print (f'objective = {model.objVal}')
print (f'bound     = {model.objBound}') 
print ('x = [')
for x_i in x:
    print (f'   {[1 if x_ij.x >= 0.5 else 0 for x_ij in x_i]}')
print (']')