
import random
import time
from deap import base, creator, tools, algorithms
import numpy as np

def Solve(items_file, node_ID):
    """
    The "individual" is a flat list where the binary decision of placing item "i" in pallet "j"
        is stored at index (i*n_pallets + j).
    The "eval_individual" function computes the total items' score in all pallets and checks
        that neither the weight nor the volume capacity of any pallet is exceeded.
        It checks also if an item is included in more than one pallet.
    """
    items_matrix_np = np.loadtxt(items_file)

    # filter items on the base
    items_matrix_np = items_matrix_np[((items_matrix_np[:,3] == node_ID))]

    # num_items_test = 250
    # items_matrix_np = items_matrix_np[:num_items_test,:]

    #  w    s    v  from to
    items_weights = items_matrix_np[ :,0]
    items_scores  = items_matrix_np[ :,1]
    items_volumes = items_matrix_np[ :,2]

    #               CG     Vol   Weight TO
    # capacities = [( 14.89, 14.8, 4500, -1), # pallet 0
    #               ( 14.89, 14.8, 4500, -1), # pallet 1
    #               (-17.57,  7.0, 3000, -1)] # pallet 17

    capacities = [( 14.89, 14.8, 4500, -1), # pallet 0
                  ( 14.89, 14.8, 4500, -1), # pallet 1
                  ( 11.47, 14.8, 4500, -1),
                  ( 11.47, 14.8, 4500, -1),
                  (  8.77, 14.8, 4500, -1),
                  (  8.77, 14.8, 4500, -1),
                  (  4.40, 14.8, 4500, -1),
                  (  4.40, 14.8, 4500, -1),
                  (  0.00, 14.8, 4500, -1),
                  (  0.00, 14.8, 4500, -1),
                  ( -4.40, 14.8, 4500, -1),
                  ( -4.40, 14.8, 4500, -1),
                  ( -8.77, 14.8, 4500, -1),
                  ( -8.77, 14.8, 4500, -1),
                  (-13.17, 14.8, 4500, -1),
                  (-13.17, 14.8, 4500, -1),
                  (-17.57, 10.0, 3000, -1),
                  (-17.57,  7.0, 3000, -1)] # pallet 17
    
    n_items     = len(items_scores)
    n_pallets = len(capacities)

    print(f"{n_items} items")

    # Evaluates the "individual", the "chromossome", a solution for a problem.
    def eval_individual(individual):
        score_sum = 0
        weight_used = [0] * n_pallets
        volume_used = [0] * n_pallets
        item_count  = [0] * n_items

        penalty = 0
        for i in range(n_items):
            for j in range(n_pallets):
                if individual[i * n_pallets + j] == 1: # if item "i" is selected for pallet "j"
                    score_sum += items_scores[i]
                    weight_used[j] += items_weights[i]
                    volume_used[j] += items_volumes[i]
                    item_count[i] += 1
        
                # Check if an item is placed in more than one pallet
                if item_count[i] > 1:
                    penalty += 200*(1 - item_count[i])

        if penalty < 0:
            return score_sum + penalty,  # Penalize if are items in more than one pallet, returns a tuple
            
        # Check if the weights and volumes are within the capacities for each pallet
        # returns a penalty (negative score) proportional to the exceed value
        for j in range(n_pallets):

            if volume_used[j] > capacities[j][1]:
                p = capacities[j][1] - volume_used[j]
                return score_sum + 100*p/capacities[j][1], # returns a tuple
    
            if weight_used[j] > capacities[j][2]:
                p = capacities[j][2] - weight_used[j]
                return score_sum + 100*p/capacities[j][2], # returns a tuple
        
        return (score_sum), # returns a tuple

    # Create classes
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    # Initialize the toolbox
    toolbox = base.Toolbox()
    toolbox.register("attr_bool", random.randint, 0, 1)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_bool, n_items * n_pallets)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("evaluate", eval_individual)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    toolbox.register("select", tools.selTournament, tournsize=3)
 
    box = 300

    # Genetic Algorithm parameters
    crossover_probability = 0.7
    mutation_probability = 0.05
    n_generations = box
    population = toolbox.population(n=box)

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("max", np.max)

    # set of best individuals (only 1 in this case)
    hall_of_fame = tools.HallOfFame(1)

    algorithms.eaSimple(population, toolbox, cxpb=crossover_probability, mutpb=mutation_probability, 
                        ngen=n_generations, stats=stats, halloffame=hall_of_fame, verbose=False)
    
    return hall_of_fame

if __name__ == "__main__":

    startTime = time.perf_counter()

    scenario = 2

    folder = "surplus20"  # 1.2
    # folder = "surplus50"  # 1.5
    # folder = "surplus100" # 2.0    

    # items: w, s, v, node, to
    items_file = f"./{folder}/scenario_{scenario}/instance_1/items.txt"  

    node_ID = 0 # 0 is the base

    hall_of_fame = Solve(items_file, node_ID)

    best_score = hall_of_fame[0].fitness.values[0]

    num_selected = sum(hall_of_fame[0])
    ind_size     = len(hall_of_fame[0])

    print(f"\nScenario {scenario}:\nSelected {num_selected} items among {ind_size} positions\nScores sum: {best_score}")

    if best_score <= 0:
        print("INFEASIBLE SOLUTION!")

    elapsed = time.perf_counter() - startTime
    print(f"{elapsed:.1f} seconds ")

    # print("----- Please execute the main py file -------------") 