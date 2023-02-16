
import random
from typing import List, Tuple

from tsp_instance import TSPInstance

def greedy_tour(instance: TSPInstance) -> Tuple[float, List]:

    tour = [0] * instance.num_nodes

    tour[0] = random.choice(tour)  # start from a random node

    remaining_nodes = set(range(0, instance.num_nodes))

    remaining_nodes.remove(tour[0])

    INF = max(instance.distances) + 10.0
    cost = 0.0
    current_node = 0
    next_node = 0
    idx = 1
    while len(remaining_nodes) > 0:
        best_dist = INF
        for j in remaining_nodes:
            dist = instance.distance(current_node, j)
            if dist < best_dist:
                best_dist = dist
                next_node = j

        cost += best_dist
        tour[idx] = next_node
        remaining_nodes.remove(next_node)
        current_node = next_node
        idx += 1
    # end while

    cost += instance.distance(tour[0], tour[-1])
    return (cost, tour)
