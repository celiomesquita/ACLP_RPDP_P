import time
import random
from utils import generate_items, generate_knapsacks
from solver import initialize_population, select_parents, uniform_crossover, batch_evaluate, mutate_in_place, adaptive_mutation_rate
import os

def main():
    # GA Parameters
    num_items      = 800
    num_kps        = 18
    pop_size       = 100
    num_gens       = 50
    elite_size     = 10 # best individuals
    batch_size     = os.cpu_count()

    # Generate items and knapsacks
    items     = generate_items(num_items)   
    knapsacks = generate_knapsacks(num_kps)

    # Initialize population
    population = initialize_population(num_items, num_kps, pop_size)
    
    start_time = time.time()  # Start timing

    for gen in range(num_gens):

        # Dynamically adjust mutation rate
        mutation_rate = adaptive_mutation_rate(gen, num_gens)        

        # Evaluate population
        fitness_values = batch_evaluate(population, knapsacks, items, batch_size)

        # # Select parents
        parents = select_parents(population, fitness_values, elite_size)

        # Generate next generation (Crossover and Mutation)
        next_generation = parents.copy()  # Start with elites

        while len(next_generation) < pop_size:
            parent1, parent2 = random.sample(parents, 2)
            offspring1, offspring2 = uniform_crossover(parent1, parent2)
            mutate_in_place(offspring1, num_kps, mutation_rate)
            mutate_in_place(offspring2, num_kps, mutation_rate)
            next_generation.extend([offspring1, offspring2])

        population = next_generation[:pop_size]  # Ensure population size remains constant

        print(f"Generation {gen+1}/{num_gens}  {fitness_values[0]:.0f}  {fitness_values[len(fitness_values)-1]:.0f} MR:{mutation_rate:.3f}")

    end_time = time.time()  # End timing

    print(f"GA completed in {end_time - start_time:.2f} seconds.")

    for individual in population[:1]:
        for item_id, knapsack_id in enumerate(individual):
            item = items[item_id]
            knapsack = knapsacks[knapsack_id - 1]  # Adjust for 0-based index
            knapsack.try_add_item(item)  # Assume try_add_item adds the item if it fits        

    vol_max = 18 * 14.
    vol_sum = 0.
    wei_max = 18 * 4500
    wei_sum = 0.
    for kp in knapsacks:
        vol_sum += kp.current_volume
        wei_sum += kp.current_weight
        print(f"{kp.knapsack_id}\t{kp.current_weight:.0f}\t{kp.current_volume:.2f}\t{len(kp.items)}")
    print(f"Ocupation rate: {wei_sum/wei_max:.3f} {vol_sum/vol_max:.3f}")

if __name__ == "__main__":
    main()
