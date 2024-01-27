import numpy as np
from deap import base, creator, tools, algorithms
import time
# import pandas as pd

def ga_deap(items_file, base_index):

    items_matrix_np = np.loadtxt(items_file)

    # filter items on the base
    items_matrix_np = items_matrix_np[((items_matrix_np[:,3] == base_index))]

    items_weights = items_matrix_np[ :,0]
    items_scores  = items_matrix_np[ :,1]
    items_volumes = items_matrix_np[ :,2]

    # Weights of items in each dimension
    weights = np.array([items_volumes,  # Dimension Volume)
                        items_weights]) # Dimension Weight)

    values = items_scores

    # pallets capacities    m3   kg
    capacities = np.array([14.8, 4500])

    # Genetic Algorithm setup                          (volumes, weights)
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    toolbox = base.Toolbox()
    toolbox.register("attr_bool", np.random.randint, 0, 2)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_bool, n=len(values))
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    def evalMKP(individual):
        weight_tot = np.dot(weights, individual)
        value_tot = np.dot(values, individual)

        # Check if the weight is greater than the capacity for any dimension
        if any(weight_tot > capacities):
            return 0,  # Return 0 if the individual exceeds capacity constraints

        return value_tot,

    toolbox.register("evaluate", evalMKP)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    toolbox.register("select", tools.selTournament, tournsize=3)

    # Genetic Algorithm parameters
    population_size = 500
    crossover_probability = 0.7
    mutation_probability = 0.05
    number_of_generations = 1500

    # Solve the MKP
    population = toolbox.population(n=population_size)

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("max", np.max)

    best = tools.HallOfFame(1)

    algorithms.eaSimple(population, toolbox, cxpb=crossover_probability, mutpb=mutation_probability, 
                        ngen=number_of_generations, stats=stats, halloffame=best, verbose=False)
    
    return best

if __name__ == "__main__":

    startTime = time.perf_counter()

    # items: w, s, v, node, to
    items_file = "./surplus20/scenario_2/instance_1/items.txt"  

    base_index = 0  

    best = ga_deap(items_file, base_index)

    # Best solution
    print(f"Best Individual = {best[0]} ({len(best[0])} items), Value = {best[0].fitness.values[0]}")

    elapsed = time.perf_counter() - startTime
    print(f"{elapsed:.1f} seconds ")

    # print("----- Please execute the main py file -------------") 