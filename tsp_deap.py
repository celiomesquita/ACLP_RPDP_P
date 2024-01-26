import numpy as np
import pandas as pd
from deap import base, creator, tools, algorithms
import time
import common

"""
São Paulo-Guarulhos International Airport (GRU)
Rio de Janeiro-Galeão International Airport (GIG)
São Paulo-Congonhas Airport (CGH)
Brasília-Presidente Juscelino Kubitschek International Airport (BSB)
Belo Horizonte-Tancredo Neves/Confins International Airport (CNF)
Porto Alegre-Salgado Filho International Airport (POA)
Recife-Guararapes/Gilberto Freyre International Airport (REC)
Salvador-Deputado Luís Eduardo Magalhães International Airport (SSA)
Fortaleza-Pinto Martins International Airport (FOR)
Curitiba-Afonso Pena International Airport (CWB)
Manaus-Eduardo Gomes International Airport (MAO)
Belém-Val de Cans/Júlio Cezar Ribeiro International Airport (BEL)
Florianópolis-Hercílio Luz International Airport (FLN)
Vitória-Eurico de Aguiar Salles Airport (VIX)
Goiânia-Santa Genoveva Airport (GYN)
"""
# 15 cities for Shims validation
CITIES = ["GRU", "GIG", "CGH", "BSB", "CNF", "POA", "REC", "SSA", "FOR", "CWB", "MAO", "BEL", "FLN", "VIX", "GYN"]

class Node(object):
    def __init__(self, id, icao):
        self.ID   = id
        self.ICAO = icao

# tour is pi in the mathematical formulation
class Tour(object):
    def __init__(self, nodes, cost):
        self.nodes = nodes
        self.cost  = cost # sum of legs costs plus CG deviation costs
        self.score   = 0.0 # sum of nodes scores
        self.elapsed  = 0 # seconds no 3D packing
        self.elapsed2 = 0 # seconds with 3D packing
        self.numOpts = 0 # sum of nodes eventual optima solutions
        self.AvgVol  = 0.0 # average ocupation rate
        self.AvgTorque  = 0.0

def getTours():

    distances_file = './params/distances15.txt'

    # Load the distance matrix from the file
    distance_matrix = pd.read_csv(distances_file, sep=' ', index_col=0)

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
    number_of_generations = 100

    # Solve the TSP
    population = toolbox.population(n=population_size)

    percent = int(len(population) * 0.2)

    ntours = 20

    if len(population) < 20:
        ntours = len(population)
    
    if percent >= 20: 
        ntours = percent
    
    hall_of_fame = tools.HallOfFame(ntours)

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("min", np.min)

    # verbose=True will print stattistics
    algorithms.eaSimple(population, toolbox, cxpb=crossover_probability, mutpb=mutation_probability, 
                        ngen=number_of_generations, stats=stats, halloffame=hall_of_fame, verbose=False)

    # # The best tours are stored in the hall of fame
    best_tours = hall_of_fame.items
    best_distances = [ind.fitness.values[0] for ind in best_tours]

    nodes = []
    for id, icao in enumerate(CITIES):
        nodes.append(Node(id, icao))

    tours = []

    for i, tour in enumerate(best_tours):
            
            tnodes = []

            for nid in tour:
                tnodes.append(nodes[nid])
              
            tours.append( Tour(tnodes, best_distances[i]) )

    return tours

if __name__ == "__main__":

    startTime = time.perf_counter()

    tours = getTours()

    for tour in tours:
        origin = tour.nodes[0]
        print(f"{origin.ICAO} ", end='')
        prev = tour.nodes[0]
        for j, node in enumerate(tour.nodes):
            if j > 0:
                print(f"{node.ICAO} ", end='')
                prev = node
        
        print(f"{origin.ICAO}", end='')
        print()


    elapsed = time.perf_counter() - startTime
    print(f"{elapsed:.1f} seconds ")

    # print("----- Please execute the main py file -------------")  