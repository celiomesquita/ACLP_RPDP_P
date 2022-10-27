# from numba import njit
import numpy as np
# from numba import int32
from timeit import default_timer as timer

# @njit
def factorial(x):
    result = 1
    for i in range(x):
        result *= i+1
    return result

# @njit
def permutations(n):
    fac = factorial(n)
    a = np.zeros((fac, n), np.uint32) # no jit
    f = 1
    for m in range(2, n+1):
        b = a[:f, n-m+1:]      # the block of permutations of range(m-1)
        for i in range(1, m):
            a[i*f:(i+1)*f, n-m] = i
            a[i*f:(i+1)*f, n-m+1:] = b + (b >= i)
        b += 1
        f *= m
    return a

def getTours(A):
    p = permutations(A)
    tours = np.zeros( ( len(p), len(p[0])+2 ), np.uint32)
    for i, row in enumerate(p):
        for j, col in enumerate(row):
            tours[i][j+1] = col+1
    return tours

# t0 = timer()

tours = getTours(3)

# t1 = timer()

# print(t1-t0)

print(tours)
