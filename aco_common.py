import math
import numpy as np

ALPHA = 1   # pheromone exponent (linearly affects attractivity)
BETA  = 4   # heuristic exponent (exponentialy affects attractivity)
NANTS = 4   # number of ants for team

# For ACO proportional selection
RNG = np.random.default_rng()

# Proportional Roulette Selection (biased if greediness > 0)
def select(values, sumVal, greediness):

    n = len(values)
    if n == 0:
        return -1
    if n == 1:
        return 0

    threshold = RNG.random()*(1.0-greediness) + greediness
    threshold *= sumVal

    pointer = 0.0
    for j, v in enumerate(values):
        pointer += v
        if pointer >= threshold: # when pointer reaches the threshold
            return j # returns the chosen index

    # just in case ....
    return int(n * RNG.random())

# calculates the pheromone to be dropped by each on ants tracks
def getDeltaTau(score, bestSoFar, numAnts):

    if bestSoFar == 0:
        return 0

    if score > bestSoFar: # better the score more pheromone is deposited
        score = bestSoFar

    dt = score / bestSoFar

    if numAnts < NANTS:
        numAnts = NANTS

    dt /= numAnts

    return dt

# each Ant makes use of "updatePheroAttract" to update pheromones
# and edge attractiveness according to the its solution value
def updatePheroAttract(score, bestSoFar, edges, numAnts):

    # evaporate some pheromone from all edges
    for id, e in enumerate(edges):
        # flatten pheromone curve
        edges[id].Pheromone = math.sqrt(e.Pheromone) / 1.25 # RHO = 0.2
        edges[id].updateAttract(ALPHA, BETA)

    deltaTau = getDeltaTau(score, bestSoFar, numAnts)

    if deltaTau < 1:
        # deposit some pheromone in the edges
        for id, e in enumerate(edges):
            if e.Pheromone < 1.0:
                edges[id].Pheromone += deltaTau
                edges[id].updateAttract(ALPHA, BETA)

# select an edge from the neighborhood by a proportional roulette wheel
def selectInNbhood(nbhood, values, sumVal, greediness):
    i = select(values, sumVal, greediness)
    e = nbhood[i]
    nbhood[i].Attract = 0
    values[i] = 0
    return e, sumVal - e.Attract    

if __name__ == "__main__":

    print("----- Please execute the main py file -------------")