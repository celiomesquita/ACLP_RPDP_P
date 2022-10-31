#!/usr/bin/env python
# coding: utf-8
import numpy as np
import random
import time
import multiprocessing as mp
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
    
def np_construct_tour( weights ):

    n = len(weights[0])

    cur_city = random.randrange(0,n)
    
    tour = np.full(shape=n, fill_value=0)

    tour[0]=cur_city
    
    free= np.zeros(n) == 0.0 
    
    free[cur_city] = False 

    i=1 
      
    while i < n: 
        
        w = weights[cur_city][free]
        
        indices = np.arange(n)[free]
        
        sel = np.argmax(w*random.random())
        
        cur_city = indices[sel] 
        
        tour[i]=cur_city 
        
        i+=1
        
        free[cur_city] = False
        
    return tour.tolist()
    
    
def construct_tours_np( weights, num_cpu ):
    tours = [None for _ in range(num_cpu)]
    for i in range(num_cpu):
        tours[i] = np_construct_tour(weights) 
    return tours

# --------------------------------
# Parallel version of the given problem using multiprocessing.
def contruct_queue(weights,q):
    q.put(np_construct_tour(weights))

def construct_tours_mp( weights, num_cpu ):
    out_queue = mp.Queue() # The object QUEUE of the module multiprocessing is assigned the variable "out_queue".
    procs = [mp.Process(target=contruct_queue,args=(weights,out_queue)) for _ in range(num_cpu)]                        
    for p in procs:
        p.start()
    tours=[out_queue.get() for _ in procs]
    return tours


def loadDistances():
    fname =  "distances.txt"      
    with open(fname, 'r') as f:
        distances = [ [ float(num) for num in line.split(' ') ] for line in f ] 
    return np.array(distances)

def printResult(method, tours, n, t):
    print(f"Valid = {check_tours(tours, n)}, {len(tours)} tours, {method} time is {1000*t:.1f} ms\n")


def printMinTourCost(tours):
    minTourCost = 999999999999.
    for i, tour in enumerate(tours):

        tourCost = 0.
        for node in tour:
            tourCost += weights[i][node]

        if minTourCost > tourCost:
            minTourCost = tourCost

    print(f"min tour cost: {minTourCost:.2f}\n")

if __name__ == '__main__':

    num_cpu = os.cpu_count()

    # weights = loadDistances()
    # n = len(weights[0])

    n = 1000 # number of cities in the problem
    weights = np.random.random((n,n)) # square matrix of random weights for edges
    
    # run the numpy version - you need to implement this
    t = time.perf_counter()
    tours = construct_tours_np( weights, num_cpu )
    t = time.perf_counter() - t
    printResult("Numpy", tours, n, t)

    printMinTourCost(tours)
    
    # run the multiprocessing version - you need to implement this
    t = time.perf_counter()
    tours = construct_tours_mp( weights, num_cpu )
    t = time.perf_counter() - t
    printResult("Multiprocessing", tours, n, t)

    printMinTourCost(tours)