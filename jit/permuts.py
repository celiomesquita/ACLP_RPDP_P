from numba import njit
import numpy as np
from numba import int32

@njit
def factorial(x):
    result = 1
    for i in range(1,x+1):
        result *= i
    return result

@njit
def permutations(n):
    fac = factorial(n)
    # a = np.zeros((fac, n), np.uint32) # no jit
    a = np.zeros((fac, n), int32)
    f = 1
    for m in range(2, n+1):
        b = a[:f, n-m+1:]      # the block of permutations of range(m-1)
        for i in range(1, m):
            a[i*f:(i+1)*f, n-m] = i
            a[i*f:(i+1)*f, n-m+1:] = b + (b >= i)
        b += 1
        f *= m
    return a

def test(A):
    for i in permutations(A):
        print(i)


r = test(4)

print(r)
