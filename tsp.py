import numpy as np
import pandas as pd
from deap import base, creator, tools, algorithms

# Load the distance matrix from the file
distance_matrix = pd.read_csv('path/to/airport_distances_brazil.txt', sep=' ', index_col=0)

# Convert the DataFrame to a numpy array for easier manipulation
distance_matrix_np = distance_matrix.to_numpy()

# Genetic Algorithm setup
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("indices", np.random.permutation, len(distance_matrix))
toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.indices)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

def evalTSP(individual):
    distance = sum(distance_matrix_np[individual[i-1], individual[i]] for i in range(len(individual)))
    return (distance + distance_matrix_np[individual[-1], individual[0]],)  # Complete the tour

toolbox.register("mate", tools.cxOrdered)
toolbox.register("mutate", tools.mutShuffleIndexes, indpb=0.05)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("evaluate", evalTSP)

# Genetic Algorithm parameters
population_size = 100
crossover_probability = 0.7
mutation_probability = 0.2
number_of_generations = 400

# Solve the TSP
population = toolbox.population(n=population_size)
hall_of_fame = tools.HallOfFame(int(len(population) * 0.2))  # Store the top 20% of all tours found

stats = tools.Statistics(lambda ind: ind.fitness.values)
stats.register("avg", np.mean)
stats.register("min", np.min)

algorithms.eaSimple(population, toolbox, cxpb=crossover_probability, mutpb=mutation_probability, 
                    ngen=number_of_generations, stats=stats, halloffame=hall_of_fame, verbose=True)

# The best tours are stored in the hall of fame
best_tours = hall_of_fame.items
best_distances = [ind.fitness.values[0] for ind in best_tours]

# Print the best tours and their distances
for tour, distance in zip(best_tours, best_distances):
    print(f"Tour: {tour}, Distance: {distance}")
