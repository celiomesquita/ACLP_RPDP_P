#!/usr/bin/env python3
import ray
import multiprocessing as mp
import numpy as np
from time import time
import multiprocessing as mp
from joblib import Parallel, delayed
from functools import partial

@ray.remote
def howmany_within_range_ray(row, minimum, maximum):
    count = 0
    for n in row:
        if minimum <= n <= maximum:
            count = count + 1
    return count

def howmany_within_range(row, minimum, maximum):
    count = 0
    for n in row:
        if minimum <= n <= maximum:
            count = count + 1
    return count    

def howmany_within_range_rowonly(row, minimum=4, maximum=8):
    count = 0
    for n in row:
        if minimum <= n <= maximum:
            count = count + 1
    return count

def howmany_within_range2(i, row, minimum, maximum):
    count = 0
    for n in row:
        if minimum <= n <= maximum:
            count = count + 1
    return (i, count)

def collect_result(result):
    global results
    results.append(result)

def printResult(method, results, elapsed):
    print(f"{method}:\t{results[:10]}\t{elapsed:.3f}s\n")

if __name__ == "__main__":

    num_cores = mp.cpu_count()

    # ray.init()

    print("Number of processors: ", num_cores)
    pool = mp.Pool(num_cores)

    # Prepare data
    np.random.RandomState(100)
    arr = np.random.randint(0, 10, size=[200000, 5])
    data = arr.tolist()

    print(f"rows {len(data)}\t columns {len(data[0])}")
    
    # serial
    start = time()
    results = []
    for row in data:
        results.append(howmany_within_range(row, minimum=4, maximum=8))
    elapsed = time() - start
    printResult("serial", results[:10], elapsed)

    # start = time()
    # pool = mp.Pool(num_cores)
    # results = [pool.apply(howmany_within_range, args=(row, 4, 8)) for row in data]
    # pool.close()
    # elapsed = time() - start
    # printResult("apply", results[:10], elapsed)

    start = time()
    pool = mp.Pool(num_cores)
    results = pool.map(howmany_within_range_rowonly, [row for row in data])
    pool.close()
    elapsed = time() - start
    printResult("map", results[:10], elapsed)

    start = time()
    pool = mp.Pool(num_cores)
    results = pool.starmap(howmany_within_range, [(row, 4, 8) for row in data])
    pool.close()
    elapsed = time() - start
    printResult("starmap", results[:10], elapsed)

    # start = time()
    # results = []
    # pool = mp.Pool(num_cores)
    # for i, row in enumerate(data):
    #     pool.apply_async(howmany_within_range2, args=(i, row, 4, 8), callback=collect_result)    
    # pool.close()
    # pool.join()  # postpones the execution of next line of code until all processes in the queue are done.   
    # elapsed = time() - start
    # results_final = [r[1] for r in results]
    # printResult("apply_async", results_final[:10], elapsed)

    # start = time()
    # funcs = []
    # for row in data:
    #     funcs.append(howmany_within_range_ray.remote(row, minimum=4, maximum=8))
    # results = ray.get(funcs)
    # elapsed = time() - start
    # printResult("ray.get", results[:10], elapsed)

    start = time()
    cc = partial(howmany_within_range_rowonly)
    output = Parallel(n_jobs=num_cores)(delayed(cc)(row) for row in data) 
    elapsed = time() - start
    printResult("joblib", results[:10], elapsed)