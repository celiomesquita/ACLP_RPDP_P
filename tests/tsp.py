#!/usr/bin/env python
# coding: utf-8
# 6G7Z1003 portfolio assignment
# python implementation of ACO tour construction using I-Roulette 
#
import numpy as np
import random
import time
import multiprocessing as mp
import ray
import os

def check_tour( tour, n ):
    # sanity check a tour to make sure it has each city represented once
    m = len(tour)
    if m != n:
        return False
    counts = [0 for i in range(n)]
    for city in tour:
        if city < 0 or city > n-1 or counts[city] != 0:
            return False
        counts[city] += 1
    return True

def check_tours( tours, n ):
    # check all tours in a list
    all_valid = True
    for t in tours:
        if not check_tour(t, n):
            all_valid = False
            break
    return all_valid

#------------------------------------
# Serial version.

def iroulette( weights ): 
    num_weights = len(weights) 
    imax = -1 
    vmax = 0 
    for i in range(num_weights):
        # the greater the weight, the more likely it is to be more than vmax
        val = weights[i] * random.random()
        if val > vmax:
            vmax = val
            imax = i
    return imax
    
def construct_tour( weights ): # The construct_tour's input is the weights matrix defined in the main() function. 
    # plain python version - construct a tour using the iroulette function
    n = len(weights[0])     # n represents the number of cities in the TSP problem. It is deduced here from 
                            # the input variable weights (equals to the number of elements of the first row 
                            # of the weights matrix).  
    # random start city
    cur_city = random.randrange(0,n)   # The start city will be chosen randomly within the range of all cities 
                                       # by using the randrange method of the library random.
    tour = [0 for _ in range(n)]
    tour[0] = cur_city

    # free[i] is true if city i has not been visited 
    free = np.zeros(n) == 0.0   # The free structure is a list of booleans which is created by using the condition 
                                # on equality of a predefined array of zeros of size n to the float 0.0 
                                # and that produces a list of booleans with the value "True". 
    free[cur_city] = False       # Set the element corresponding to the index "cur_city" in the free array to "False" 
                                 # as this is the start city (i.e., visisted city). 
        
    i=1
    while i < n:
        w = weights[cur_city][free]        
        indices = np.arange(n)[free]

        sel = iroulette(w)   # sel represents the position corresponding to the maximum product "weight*random" which is
                             # evaluated by calling the function "iroulette". 
        cur_city = indices[sel] # the variable "cur_city" is updated by the next city to be visited 

        tour[i] = cur_city
        i += 1
        free[cur_city] = False  # The free array is updated by assigning the value False to the index 
                                # corresponding to the cur_city.
    return tour # The output of this construct_tour function is the tour list. 

def construct_tours( weights, num_ants ):
    tours = [None for _ in range(num_ants)]
    for i in range(num_ants):
        tours[i] = construct_tour(weights) 
    return tours

# --------------------------------------------------------
# Vectorised version of the given problem using Numpy.

def np_construct_tour( weights ):
# todo: make this construct a tour from the weights, using a vectorized version of I-Roulette
    n = len(weights[0])     # n represents the number of cities in the TSP problem. It is deduced here from 
                            # the input variable weights (equals to the number of elements of the first row 
                            # of the weights matrix).  
    # random start city
    cur_city = random.randrange(0,n)   # The start city will be chosen randomly within the range of all cities 
                                       # by using the randrange method of the library random.
    tour = np.full(shape=n, fill_value=0)

    tour[0]=cur_city    # The cur_city is assigned to the first element of the array tour. 
    # free[i] is true if city i has not been visited 
    free= np.zeros(n) == 0.0  # The free structure is a list of booleans which is created 
                              # by using the condition on equality of a predefined array of zeros of size n 
                              # to the float 0.0 and that produces a list of booleans with the value "True".
    free[cur_city] = False   # Set the element corresponding to the index "cur_city" in the free array to "False" 
                             # as this is the start city (i.e., visited city). 

    i=1  # i is a counter that will help to assign cities to the tour array.  
    while i < n: # Evaluates true if there are still cities that have not been visited. 
        w = weights[cur_city][free] # w is obtained by boolean indexing the list "weights[cur_city]" using the list "free". 
        indices = np.arange(n)[free] # indices is obtained by boolean indexing, using the list "free", the list of integers 
                                     # (0 to n-1) generated by the numpy built-in "np.arange".
        sel = np.argmax(w*random.random()) # The variable sel is vectorised by using the numpy built-in "np.argmax" 
                                           # to return the position that corresponds to the maximum value 
                                           # within the output of the product "w*random.random()".
        cur_city = indices[sel] # the variable "cur_city" is updated by the next city to be visited 
                                # which is selected by indexing the indices list using the "sel" variable.
        tour[i]=cur_city # the new "cur_city" is assigned to the next element (element i) of the tour array. 
        i+=1 # The counter i is incremented by 1.
        free[cur_city] = False  # The free array is updated by assigning the value False to the index 
                                # corresponding to the cur_city.
    return tour.tolist() # The data structure of the output "tour" (numpy array) is converted to a list to show similarity 
                         # with the output of the serial code. 
    
def construct_tours_np( weights, num_ants ):
    tours = [None for _ in range(num_ants)]
    for i in range(num_ants):
        tours[i] = np_construct_tour(weights) 
    return tours

# --------------------------------
# Parallel version of the given problem using multiprocessing.
def contruct_queue(weights,q):
    q.put(np_construct_tour(weights))

def construct_tours_mp( weights, num_ants ):
    out_queue = mp.Queue() # The object QUEUE of the module multiprocessing is assigned the variable "out_queue".
    procs = [mp.Process(target=contruct_queue,args=(weights,out_queue)) for _ in range(num_ants)]                        
    for p in procs:
        p.start()
    tours=[out_queue.get() for _ in procs]
    return tours


# --------------------------------------------------------
# Vectorised version of the given problem using Numpy and Ray.
@ray.remote
def np_construct_tour_ray( weights ):
    return np_construct_tour( weights )

# Parallel version of the given problem using Ray.
def construct_tours_ray( weights, num_ants ):
    tours = ray.get([np_construct_tour_ray.remote(weights) for _ in range(num_ants)]) 
    return tours  
# --------------------------------------------------------


def loadDistances():
    fname =  "distances.txt"      
    with open(fname, 'r') as f:
        distances = [ [ float(num) for num in line.split(' ') ] for line in f ] 
    return np.array(distances)

def printResult(method, tours, n, t):
    print(f"Valid = {check_tours(tours, n)}, {len(tours)} tours, {method} time is {1000*t:.1f} ms\n")

#---------------------------------
def main():

    num_ants = os.cpu_count()


    # weights = loadDistances()
    # n = len(weights[0])

    n = 1000 # number of cities in the problem
    weights = np.random.random((n,n)) # square matrix of random weights for edges

    # run the plain python version
    t = time.perf_counter()
    tours = construct_tours( weights, num_ants)
    t = time.perf_counter() - t
    printResult("Serial", tours, n, t)
    
    # run the numpy version - you need to implement this
    t = time.perf_counter()
    tours = construct_tours_np( weights, num_ants )
    t = time.perf_counter() - t
    printResult("Numpy", tours, n, t)
    
    # run the multiprocessing version - you need to implement this
    t = time.perf_counter()
    tours = construct_tours_mp( weights, num_ants )
    t = time.perf_counter() - t
    printResult("Multiprocessing", tours, n, t)

    # run the ray version
    ray.init(num_cpus=num_ants)
    t = time.perf_counter()
    tours = construct_tours_ray( weights, num_ants )
    t = time.perf_counter() - t
    printResult("Ray", tours, n, t)


if __name__ == '__main__':
    main()
