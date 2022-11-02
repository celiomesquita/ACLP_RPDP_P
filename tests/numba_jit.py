import math
from numba import njit
import timeit

@njit
def std(xs):
    # compute the mean
    mean = 0
    for x in xs: 
        mean += x
        mean /= len(xs)
    # compute the variance
    ms = 0
    for x in xs:
        ms += (x-mean)**2
        variance = ms / len(xs)
        std = math.sqrt(variance)
    return std

SETUP = """
import math
import numpy as np
from __main__ import std
"""

CODE = """
a = np.random.normal(0, 1, 10_000_000)
std(a)
"""

if __name__=="__main__":

    t1 = timeit.Timer(CODE, SETUP)

    ntimes = 3

    t2 = t1.timeit(number=ntimes)

    print(f"{t2/ntimes:.2f} seconds\n") 
                    