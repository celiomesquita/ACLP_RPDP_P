import random
from utils import generate_items, generate_knapsacks
from multiprocessing import Pool

def initialize_population(item_count, knapsack_count, population_size):
    #        individual
    return [[random.randint(1, knapsack_count) for _ in range(item_count)] for _ in range(population_size)]

def fitness(individual, knapsacks, items):
    # Reset knapsacks
    for knapsack in knapsacks:
        knapsack.reset()  # Assume reset method clears items and recalculates capacity
    
    # Assign items to knapsacks based on the individual's genes
    for item_id, knapsack_id in enumerate(individual):
        item = items[item_id]
        knapsack = knapsacks[knapsack_id - 1]  # Adjust for 0-based index
        knapsack.try_add_item(item)  # Assume try_add_item adds the item if it fits
    
    # Calculate and return the total value across all knapsacks
    total_value = sum(k.total_value() for k in knapsacks)
    return total_value

def select_parents(population, fitness_values, elite_size=10):
    """
    Selects parents for the next generation using elitism and tournament selection.

    :param population: The current population.
    :param fitness_values: List of fitness_values.
    :param elite_size: Number of top individuals to automatically pass on to the next generation.
    :return: List of individuals selected as parents.
    """
    # Calculate fitness for each individual
    fitness_results = [(individual, fitness_values) for individual in population]

    # Sort by fitness (higher is better)
    sorted_population = sorted(fitness_results, key=lambda x: x[1], reverse=True)

    # Select elite
    elites = [individual for individual, _ in sorted_population[:elite_size]]

    # Tournament selection or other selection mechanism could be added here for the rest

    return elites

def uniform_crossover(parent1, parent2):
    offspring1, offspring2 = parent1.copy(), parent2.copy()
    for i in range(len(parent1)):
        if random.random() < 0.5:  # Swap genes with 50% probability
            offspring1[i], offspring2[i] = offspring2[i], offspring1[i]
    return offspring1, offspring2

def mutate_in_place(individual, knapsack_count, mutation_rate):
    for i in range(len(individual)):
        if random.random() < mutation_rate:
            # Directly modify the individual's gene
            individual[i] = random.randint(1, knapsack_count)

def evaluate_batch(batch, items, knapsacks):
    """Evaluates a batch of individuals for their fitness."""
    return [fitness(individual, knapsacks, items) for individual in batch]

def batch_evaluate(population, knapsacks, items, batch_size=10):
    """Splits the population into batches and evaluates them in parallel."""
    batches = [population[i:i + batch_size] for i in range(0, len(population), batch_size)]
    pool = Pool(processes=16)
    results = pool.starmap(evaluate_batch, [(batch, items, knapsacks) for batch in batches])
    pool.close()
    pool.join()
    # Flatten the list of lists to a single list of fitness values
    fitness_values = [fit for sublist in results for fit in sublist]
    return fitness_values


def adaptive_mutation_rate(gen, num_gens):
    # Example: Linearly decrease the mutation rate over generations
    initial_rate = 0.20
    final_rate   = 0.01
    rate = initial_rate - (gen / num_gens) * (initial_rate - final_rate)
    return max(rate, final_rate)

# NOT USED YET

def rank_selection(population, fitnesses, selection_size):
    ranked_population = sorted(zip(population, fitnesses), key=lambda x: x[1], reverse=True)
    # Implement selection based on ranking here
    # This is a placeholder for the logic you'll need to implement

