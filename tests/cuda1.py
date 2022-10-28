from numba import cuda
import numpy
import math

# https://nyu-cds.github.io/python-numba/05-cuda/

# CUDA kernel
@cuda.jit
def my_kernel(io_array):
    pos = cuda.grid(1)
    if pos < io_array.size:
        io_array[pos] *= 2 # do the computation

# Host code 
if __name__=="__main__": 

    data = numpy.ones(256)
    threadsperblock = 256
    blockspergrid = math.ceil(data.shape[0] / threadsperblock)
    my_kernel[blockspergrid, threadsperblock](data)
    print(data)