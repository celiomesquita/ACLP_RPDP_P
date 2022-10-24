from numba import cuda, jit
import numpy as np
from timeit import default_timer as timer

# To run on CPU
def func(a):
    for i in range(len(a)):
        a[i] += 1

# To run on GPU
@jit
def func2(x):
    return x+1

if __name__=="__main__":

    n = 10000000
    a = np.ones(n, dtype = np.float64)

    start = timer()

    func(a)
    print(f"CPU {timer()-start:.2f}s")

    start = timer()

    func2(a)
    cuda.profile_stop()
    print(f"GPU {timer()-start:.2f}s")