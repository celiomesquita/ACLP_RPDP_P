import random
import multiprocessing as mp
from multiprocessing import Pool


def initialize_population(item_count, knapsack_count, population_size):
    #        a random knapsack                   for each item
    return [[random.randint(0, knapsack_count-1) for _ in range(item_count)] for _ in range(population_size)]

def fitness(individual, knapsacks, items, torque, solDict, itemsDict, N):
    # Reset knapsacks
    for knapsack in knapsacks:
        knapsack.reset()  # Assume reset method clears items and recalculates capacity
    
    try:
        # Assign items to knapsacks based on the individual's genes
        #   position    value
        for item_id, knapsack_id in enumerate(individual):
            item     = items[item_id]
            knapsack = knapsacks[knapsack_id]
            knapsack.try_add_item(item, torque, solDict, itemsDict, N)
    
    except IndexError as e:
        # Log error context for debugging
        print(f"Error accessing index {individual[0]} in 'items' with length {len(items)}")
        raise e  # Re-raise exception to maintain original error behavior

    # Calculate and return the total value across all knapsacks
    total_value = sum(kp.total_value() for kp in knapsacks)
    return total_value

def select_parents(population, fitness_values, elite_size=10):
    """
    Selects parents for the next generation using elitism or tournament selection.
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
            # a random knapsack for this item
            individual[i] = random.randint(0, knapsack_count-1)

def adaptive_mutation_rate(gen, num_gens):
    # Example: Linearly decrease the mutation rate over generations
    initial_rate = 0.20
    final_rate   = 0.01
    rate = initial_rate - (gen / num_gens) * (initial_rate - final_rate)
    return max(rate, final_rate)

def evaluate_batch(batch, knapsacks, items, torque, solDict, itemsDict, N, output_queue):
    """Evaluates a batch of individuals for their fitness and stores results in the output queue."""
    # Calculate fitness for the batch and put the result in the output queue
    output_queue.put([fitness(individual, knapsacks, items, torque, solDict, itemsDict, N) for individual in batch])

def batch_evaluate(population, knapsacks, items, torque, solDict, itemsDict, N, batch_size=10):

    """Splits the population into batches and evaluates them in parallel using multiprocessing.Process."""

    # batch_size = 1

    batches = [population[i:i + batch_size] for i in range(0, len(population), batch_size)]

    processes = []
    output_queue = mp.Queue()
    
    # Create and start a process for each batch
    for batch in batches:
        process = mp.Process(target=evaluate_batch, args=(batch, knapsacks, items, torque, solDict, itemsDict, N, output_queue))
        processes.append(process)
        process.start()
    
    # Wait for all processes to complete
    for process in processes:
        process.join()
    
    # Collect all fitness values from the output queue
    fitness_values = []
    while not output_queue.empty():
        fitness_values.extend(output_queue.get())
    
    return fitness_values

def evaluate_population(population, knapsacks, items, torque, solDict, itemsDict, N):
    """
    Evaluates the entire population in parallel using multiprocessing.
    """
    
    # Prepare a list of arguments for the fitness function
    args = [(individual, knapsacks, items, torque, solDict, itemsDict, N) for individual in population]

    # Create a pool of worker processes
    with Pool() as pool:
        # Map each individual to the fitness function and collect the results
        results = pool.starmap(fitness, args)

    return results


# NOT USED YET

def rank_selection(population, fitnesses, selection_size):
    ranked_population = sorted(zip(population, fitnesses), key=lambda x: x[1], reverse=True)
    # Implement selection based on ranking here
    # This is a placeholder for the logic you'll need to implement

