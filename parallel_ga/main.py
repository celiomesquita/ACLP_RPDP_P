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
    items = generate_items(num_items)

    dists_cg = [14.89,
                14.89,
                11.47,
                11.47,
                8.77,
                8.77,
                4.40,
                4.40,
                0.00,
                0.00,
                -4.40,
                -4.40,
                -8.77,
                -8.77,
                -13.17,
                -13.17,
                -17.57,
                -17.57]

    knapsacks = generate_knapsacks(dists_cg, num_kps)

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

        print(f"Gen.: {gen+1}/{num_gens}  mutation rate:{mutation_rate:.3f}")

    end_time = time.time()  # End timing

    print(f"GA completed in {end_time - start_time:.2f} seconds.")

    for individual in population[:1]:
        print("---")
        for item_id, knapsack_id in enumerate(individual):
            item = items[item_id]
            knapsack = knapsacks[knapsack_id - 1]  # Adjust for 0-based index
            knapsack.try_add_item(item)  # Assume try_add_item adds the item if it fits        

    vol_max = num_kps * 14.
    vol_sum = 0.
    wei_max = num_kps * 4500
    wei_sum = 0.
    num_items = 0
    for kp in knapsacks:
        vol_sum += kp.current_volume
        wei_sum += kp.current_weight
        num_items += len(kp.items)
        kp.print_results()

    print(f"Ocupation: {wei_sum/wei_max:.3f} {vol_sum/vol_max:.3f} items:{num_items}")

if __name__ == "__main__":
    main()
