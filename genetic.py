import time
import random
from parallel_ga import solver as ga_solver
from parallel_ga import models as ga_models
import os
import multiprocessing as mp
import numpy as np

def Solve(pallets, items2, cfg, pi, k, secBreak, nodeTorque, solDict, itemsDict):

    startTime = time.perf_counter()

    N = len(items2)
    M = len(pallets)

    print(f"\nGenetic Algorithm for ACLP+RPDP ({pi}-{k})")        
    print(f"{N} items  {M} pallets")

    # GA Parameters
    num_items      = N
    num_kps        = M
    pop_size       = 200
    num_gens       = 100
    elite_size     = 10 # best individuals
    batch_size     = os.cpu_count()
    torque         = {'maximum': cfg.maxTorque, 'current': nodeTorque.value}

    solMatrix_ga = mp.Array('i', [0 for _ in np.arange(N*M)] )
    mpItems_ga   = mp.Array('i', [0 for _ in np.arange(N)] ) # to check items inclusions feasibility

    solDict_ga   = dict(solMatrix=solMatrix_ga)
    itemsDict_ga = dict(mpItems=mpItems_ga)       

    items = []
    id = 0
    for it in items2:
        item = ga_models.Item(id, it.W, it.V, it.S, it.To)
        items.append(item)
        id += 1
    
    knapsacks = []
    id = 0
    for p in pallets:
        kp = ga_models.Knapsack(p.D, id, p.W, p.V, p.Dest[k])
        knapsacks.append(kp)
        id += 1

    # Initialize population
    population = ga_solver.initialize_population(num_items, num_kps, pop_size)
    
    # start_time = time.time()  # Start timing

    for gen in range(num_gens):

        if (time.perf_counter() - startTime) > secBreak:
            break

        # Dynamically adjust mutation rate
        mutation_rate = ga_solver.adaptive_mutation_rate(gen, num_gens)   

        # fitness_values = ga_solver.evaluate_population(population, knapsacks, items, torque, N)   

        # batch_size = len(items)  

        # Evaluate population
        fitness_values = ga_solver.batch_evaluate( population, knapsacks, items, torque, solDict_ga, itemsDict_ga, N, batch_size)

        # # Select parents
        parents = ga_solver.select_parents(population, fitness_values, elite_size)

        # Generate next generation (Crossover and Mutation)
        next_generation = parents.copy()  # Start with elites

        while len(next_generation) < pop_size:
            parent1, parent2 = random.sample(parents, 2)
            offspring1, offspring2 = ga_solver.uniform_crossover(parent1, parent2)
            ga_solver.mutate_in_place(offspring1, num_kps, mutation_rate)
            ga_solver.mutate_in_place(offspring2, num_kps, mutation_rate)
            next_generation.extend([offspring1, offspring2])

        population = next_generation[:pop_size]  # Ensure population size remains constant

        # print(f"Gen.: {gen+1}/{num_gens}  mutation rate:{mutation_rate:.3f}")

    # end_time = time.time()  # End timing

    # print(f"GA completed in {end_time - start_time:.2f} seconds.")
        

    for individual in population[:1]:

        ga_solver.fitness(individual, knapsacks, items, torque, solDict, itemsDict, N)

        # for item_id, knapsack_id in enumerate(individual):
        #     item = items[item_id]
        #     knapsack = knapsacks[knapsack_id]
        #     knapsack.try_add_item(item, torque, solDict_ga, itemsDict_ga, N)

    # vol_max = num_kps * 14.
    # vol_sum = 0.
    # wei_max = num_kps * 4500
    # wei_sum = 0.
    # num_items = 0
    # print("Kp\tWeight\tVolume\tItems")
    # for kp in knapsacks:
    #     vol_sum += kp.current_volume
    #     wei_sum += kp.current_weight
    #     num_items += len(kp.items)
    #     kp.print_results()

    # print(f"---\nOcup:\t{wei_sum/wei_max:.3f}\t{vol_sum/vol_max:.3f}\t{num_items}")

    # tau = float(torque["current"]/torque["maximum"])
    # print(f"Relative node torque: {tau:.2f}")  

if __name__ == "__main__":
    Solve()