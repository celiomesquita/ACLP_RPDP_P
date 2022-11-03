#!/usr/bin/env python3
import os
import time
import multiprocessing as mp
import ray
from joblib import Parallel, delayed

NCPU = os.cpu_count()


# Normal Python
def fibonacci_local(sequence_size):
    fibonacci = []
    for i in range(0, sequence_size):
        if i < 2:
            fibonacci.append(i)
            continue
        fibonacci.append(fibonacci[i-1]+fibonacci[i-2])
    return sequence_size

# Ray task
@ray.remote
def fibonacci_distributed(sequence_size):
    fibonacci = []
    for i in range(0, sequence_size):
        if i < 2:
            fibonacci.append(i)
            continue
        fibonacci.append(fibonacci[i-1]+fibonacci[i-2])
    return sequence_size


def printResult(method, elapsed):
    print(f"{method}:\t{elapsed:.3f}s")


# Normal Python
def run_local(sequence_size):
    start_time = time.time()
    results = [fibonacci_local(sequence_size) for _ in range(NCPU-1)]
    duration = time.time() - start_time
    printResult("Local", duration)

# Ray
def run_remote(sequence_size):
    start_time = time.time()
    results = ray.get([fibonacci_distributed.remote(sequence_size) for _ in range(NCPU-1)])
    duration = time.time() - start_time
    printResult("Remote", duration)

def queue_put(sequence_size, q):
    q.put(fibonacci_local(sequence_size)) 
   
# Multiprocessing
def run_mp(sequence_size):

    start_time = time.time()

    out_queue = mp.Queue()

    procs = [mp.Process(target=queue_put,args=(sequence_size, out_queue)) for _ in range(NCPU-1)]

    for p in procs:
        p.start()

    results = [out_queue.get() for _ in procs]

    duration = time.time() - start_time
    printResult("Multipro", duration)

# joblib parallel
def run_joblib(sequence_size):

    start_time = time.time()

    output = Parallel(n_jobs=NCPU-1)(delayed(fibonacci_local)(sequence_size) for _ in range(NCPU-1)) 

    duration = time.time() - start_time
    printResult("joblib", duration)

if __name__ == "__main__":

    ray.init(num_cpus=NCPU-1)

    size = 150000

    run_local(size)
    run_remote(size)
    run_mp(size)
    run_joblib(size)
