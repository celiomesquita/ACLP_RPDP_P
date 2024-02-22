import random
from models import Item, Knapsack

def generate_items(n=800):
    """
    Generates a list of items with random weights, volumes, and values.

    :param n: Number of items to generate.
    :return: List of Item objects.
    """
    items = []
    for i in range(n):
        weight = random.uniform(10, 340)   # Assuming weight range
        volume = random.uniform(0.05, 1.0) # Assuming volume range
        value = random.uniform(10, 100)    # Assuming value range
        items.append(Item(item_id=i, weight=weight, volume=volume, value=value))
    return items

def generate_knapsacks(dists_cg, n=18, max_weight=4500, max_volume=14 ):
    """
    Generates a list of knapsacks with specified maximum weight and volume capacities.

    :param n: Number of knapsacks to generate.
    :param max_weight: Maximum weight capacity for each knapsack.
    :param max_volume: Maximum volume capacity for each knapsack.
    :return: List of Knapsack objects.
    """
    knapsacks = [Knapsack(dists_cg[i], knapsack_id=i, max_weight=max_weight, max_volume=max_volume ) for i in range(n)]
    return knapsacks

def read_items_from_file(filepath):
    """
    Reads items from a file. Each line in the file should represent an item with its weight, volume, and value.

    :param filepath: Path to the file containing items data.
    :return: List of Item objects.
    """
    items = []
    with open(filepath, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            if len(parts) == 4:
                item_id, weight, volume, value = map(float, parts)
                items.append(Item(item_id=int(item_id), weight=weight, volume=volume, value=value))
    return items

def read_knapsacks_from_file(filepath):
    """
    Reads knapsack capacities from a file. Each line represents a knapsack with its maximum weight and volume.

    :param filepath: Path to the file containing knapsacks data.
    :return: List of Knapsack objects.
    """
    knapsacks = []
    with open(filepath, 'r') as file:
        for i, line in enumerate(file):
            parts = line.strip().split(',')
            if len(parts) == 2:
                max_weight, max_volume = map(float, parts)
                knapsacks.append(Knapsack(knapsack_id=i, max_weight=max_weight, max_volume=max_volume))
    return knapsacks
