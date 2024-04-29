import random
from parallel_ga import models

def generate_items(n=800):
    """
    Generates a list of items with random weights, volumes, and values.
    """
    items = []
    for i in range(n):
        weight = random.uniform(10, 340)   # Assuming weight range
        volume = random.uniform(0.05, 1.0) # Assuming volume range
        value = random.uniform(10, 100)    # Assuming value range
        items.append(models.Item(item_id=i, weight=weight, volume=volume, value=value))
    return items

def generate_knapsacks(dists_cg, n=18, max_weight=4500, max_volume=14 ):
    """
    Generates a list of knapsacks with specified maximum weight and volume capacities.
    """
    knapsacks = [models.Knapsack(dists_cg[i], id=i, max_weight=max_weight, max_volume=max_volume ) for i in range(n)]
    return knapsacks
