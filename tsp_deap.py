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

def rotate(tour_nodes, base_node):
    """
    Shifts the elements of the list such that the specified base_node becomes the first element.
    The rest of the list follows in the original order.
    If the base_node is not found in the list, the original list is returned.
    """
    if base_node in tour_nodes:
        index = tour_nodes.index(base_node)
        return tour_nodes[index:] + tour_nodes[:index]
    else:
        return tour_nodes



def getTours(distances_file, numNodes):   

    # Load the distance matrix from the file
    distance_matrix = pd.read_csv(distances_file, sep=' ', index_col=0)

    cities = distance_matrix.columns.to_list()[:numNodes]

    distance_matrix = distance_matrix.head(numNodes)

    distance_matrix = pd.DataFrame(distance_matrix, columns = cities)

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

    # ntours = len(population)

    # percent = int(ntours * 0.3)
    
    # if percent >= 20: 
    #     ntours = percent

    # print(f"DEAP: The hall of fame has {ntours} tours.")
        
    # hall_of_fame = tools.HallOfFame(ntours)

    hall_of_fame = tools.HallOfFame(len(population))

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
    for id, icao in enumerate(cities):
        nodes.append(common.Node(id, icao))

    tours = []

    for i, tour in enumerate(best_tours):
         
        tnodes = []

        for nid in tour:
            tnodes.append(nodes[nid])

        tours.append( common.Tour(tnodes, best_distances[i]) )

    return tours # list of tours

def list_to_string(lst):
    """
    Converts a list into a string with elements separated by spaces.
    """
    return ' '.join(map(str, lst))

if __name__ == "__main__":

    startTime = time.perf_counter()

    distances_file = './params/distances15.txt'
    # distances_file = './params/distances7.txt'

    tours = getTours(distances_file, 15)

    base_node = "GRU"

    for tour in tours:

        icaos = []
        for n in tour.nodes:
            icaos.append(n.ICAO)

        icaos = rotate(icaos, base_node)

        print(icaos, tour.cost)

        break

        # origin = tour.nodes[0]
        # stour = f"{origin.ICAO} "
        # prev = tour.nodes[0]
        # for j, node in enumerate(tour.nodes):
        #     if j > 0:
        #         stour += f"{node.ICAO} "
        #         prev = node
        
        # print(stour)


    elapsed = time.perf_counter() - startTime
    print(f"{elapsed:.1f} seconds ")

    # print("----- Please execute the main py file -------------")  