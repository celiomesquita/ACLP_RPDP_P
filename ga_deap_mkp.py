import numpy as np
from deap import base, creator, tools, algorithms
import time
import pandas as pd

# ga_deap.Solve( pallets, items, cfg, k, node.tLim, nodeTorque, solDict, itemsDict) 
def Solve(items_file, base_index):

    # pallets capacities    m3   kg
    capacities = np.array([14.8, 4500])
    # capacities = np.array([5, 450])    

    items_matrix_np = np.loadtxt(items_file)

    # #                             w    s    v  from to
    # items_matrix_np = np.array([[251, 10, 1.696, 0, 1],
    #                             [15, 40, 0.095, 0, 1],
    #                             [106, 5, 0.440, 0, 2],
    #                             [15, 10, 0.076, 0, 1],
    #                             [66, 22, 0.262, 0, 1],
    #                             [206, 40, 0.713, 0, 2],
    #                             [45, 70, 0.246, 0, 1],
    #                             [122, 70, 0.726, 0, 1],
    #                             [73, 52, 0.302, 0, 1],
    #                             [36, 10, 0.110, 0, 2],
    #                             [27, 30, 0.088, 0, 2],
    #                             [14, 22, 0.074, 0, 1],
    #                             [25, 40, 0.104, 0, 2],
    #                             [10, 70, 0.047, 0, 2],
    #                             [10, 100, 0.031, 0, 1]]  )

    # print(items_matrix_np[:5,:])

    items_matrix_np = items_matrix_np[:160,:]

    # filter items on the base
    items_matrix_np = items_matrix_np[((items_matrix_np[:,3] == base_index))]

    items_weights = items_matrix_np[ :,0]
    items_scores  = items_matrix_np[ :,1]
    items_volumes = items_matrix_np[ :,2]

    # Weights of items in each dimension
    weights = np.array([items_volumes,  # Dimension Volume)
                        items_weights]) # Dimension Weight)

    values = items_scores



    # Genetic Algorithm setup
    creator.create("FitnessMax", base.Fitness, weights=(1,))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    toolbox = base.Toolbox()
    toolbox.register("attr_bool", np.random.randint, 0, 2)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_bool, n=len(values))
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    def evalMKP(individual):
        weight_tot = np.dot(weights, individual) # sum product
        value_tot  = np.dot(values,  individual)

        # Check if the weight is greater than the capacity for any dimension
        if any(weight_tot > capacities):
            return 0,  # Return 0 if the individual exceeds capacity constraints

        return value_tot,

    toolbox.register("evaluate", evalMKP)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    toolbox.register("select", tools.selTournament, tournsize=3)

    # Genetic Algorithm parameters
    population_size = 200
    crossover_probability = 0.7
    mutation_probability = 0.05
    number_of_generations = 400

    # Solve the MKP
    population = toolbox.population(n=population_size)

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("max", np.max)

    hall_of_fame = tools.HallOfFame(1)

    algorithms.eaSimple(population, toolbox, cxpb=crossover_probability, mutpb=mutation_probability, 
                        ngen=number_of_generations, stats=stats, halloffame=hall_of_fame, verbose=False)
    
    best_scores = [ind.fitness.values[0] for ind in hall_of_fame.items]

    return hall_of_fame, best_scores

if __name__ == "__main__":

    startTime = time.perf_counter()

    scenario = 2

    folder = "surplus20"  # 1.2
    # folder = "surplus50"  # 1.5
    # folder = "surplus100" # 2.0    

    # items: w, s, v, node, to
    items_file = f"./{folder}/scenario_{scenario}/instance_1/items.txt"  

    base_index = 0  

    hall_of_fame, best_scores = Solve(items_file, base_index)

    for i, b in enumerate(hall_of_fame):

        num_selected = sum(hall_of_fame[i])

        print(f"\n{i+1} Scenario {scenario}:\nSelected {num_selected} in {len(hall_of_fame[i])} items\nScores sum: {best_scores[i]}")

    elapsed = time.perf_counter() - startTime
    print(f"{elapsed:.1f} seconds ")

    # print("----- Please execute the main py file -------------") 