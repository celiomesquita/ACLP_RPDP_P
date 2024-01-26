import numpy as np
from deap import base, creator, tools, algorithms

# Example MKP data
# Values of items
values = np.array([60, 100, 120])
# Weights of items in each dimension
weights = np.array([[10, 20, 30],  # Dimension 1 weights (volumes)
                    [20, 15, 25]]) # Dimension 2 weights (weights)
# Capacities of the knapsack in each dimension
#                      m3   kg
capacities = np.array([50, 50])

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
population_size = 50
crossover_probability = 0.7
mutation_probability = 0.2
number_of_generations = 100

# Solve the MKP
population = toolbox.population(n=population_size)

stats = tools.Statistics(lambda ind: ind.fitness.values)
stats.register("avg", np.mean)
stats.register("max", np.max)

best = tools.HallOfFame(1)

algorithms.eaSimple(population, toolbox, cxpb=crossover_probability, mutpb=mutation_probability, 
                    ngen=number_of_generations, stats=stats, halloffame=best, verbose=True)

# Best solution
print(f"Best Individual = {best[0]}, Value = {best[0].fitness.values[0]}")
