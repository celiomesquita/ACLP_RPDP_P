import numpy as np
import numba as nb
from numba import cuda
from timeit import default_timer as timer

import logging
logger = logging.getLogger("numba")
logger.setLevel(logging.ERROR)
logging.disable(logging.WARNING)

# cuda.detect()

@cuda.jit
def add_scalars(a, b, c):
    c[0] = a + b

if __name__=="__main__":

    a = 2.
    b = 7.

    dev_c = cuda.device_array(1, float)

    add_scalars[1, 1](a, b, dev_c)

    c = dev_c.copy_to_host()
    print(f"{a} + {b} = {c[0]}")

"""
@cuda.jit
def add_array(a, b, c):
    i = cuda.grid(1)
    if i < a.size:
        c[i] = a[i] + b[i]

N = 1_000_000
a = np.arange(N, dtype=np.float32)
b = np.arange(N, dtype=np.float32)

dev_a = cuda.to_device(a)
dev_b = cuda.to_device(b)
dev_c = cuda.device_array_like(a)

threads_per_block = 256
blocks_per_grid = (N + (threads_per_block - 1)) // threads_per_block
# Note that
#     blocks_per_grid == ceil(N / threads_per_block)
# ensures that blocks_per_grid * threads_per_block >= N

add_array[blocks_per_grid, threads_per_block](dev_a, dev_b, dev_c)

c = dev_c.copy_to_host()
np.allclose(a + b, c)
"""
#  True

"""
# Example 1.3: Add arrays with cuda.grid
@cuda.jit
def add_array(a, b, c):
    i = cuda.grid(1)
    if i < a.size:
        c[i] = a[i] + b[i]

N = 20
a = np.arange(N, dtype=np.float32)
b = np.arange(N, dtype=np.float32)

dev_a = cuda.to_device(a)
dev_b = cuda.to_device(b)
dev_c = cuda.device_array_like(a)

add_array[4, 8](dev_a, dev_b, dev_c)

c = dev_c.copy_to_host()
print(c)

#  [ 0.  2.  4.  6.  8. 10. 12. 14. 16. 18. 20. 22. 24. 26. 28. 30. 32. 34. 36. 38.]
"""


"""
# Example 1.2: Add arrays
@cuda.jit
def add_array(a, b, c):
    i = cuda.threadIdx.x + cuda.blockDim.x * cuda.blockIdx.x
    if i < a.size:
        c[i] = a[i] + b[i]

N = 20
a = np.arange(N, dtype=np.float32)
b = np.arange(N, dtype=np.float32)
dev_c = cuda.device_array_like(a)

add_array[4, 8](a, b, dev_c)

c = dev_c.copy_to_host()
print(c)

#  [ 0.  2.  4.  6.  8. 10. 12. 14. 16. 18. 20. 22. 24. 26. 28. 30. 32. 34. 36. 38.]
"""





# # To run on CPU
# def func(a):
#     for i in range(len(a)):
#         a[i] += 1
#     return sum(a)

# # To run on GPU
# @cuda.jit
# def func2(x):
#     return x+1

# @cuda.jit
# def my_kernel(io_array):
#     # Thread id in a 1D block
#     tx = cuda.threadIdx.x
#     # Block id in a 1D grid
#     ty = cuda.blockIdx.x
#     # Block width, i.e. number of threads per block
#     bw = cuda.blockDim.x
#     # Compute flattened index inside the array
#     pos = tx + ty * bw
#     if pos < io_array.size:  # Check array boundaries
#         io_array[pos] *= 2 # do the computation

# if __name__=="__main__":

#     print(cuda.gpus) # <Managed Device 0>

#     # Launch a terminal shell and type the commands for a machine without CUDA:
#     # export NUMBA_ENABLE_CUDASIM=1    

#     # Create the data array - usually initialized some other way
#     data = np.ones(256)

#     # Set the number of threads in a block
#     threadsperblock = 32 

#     # Calculate the number of thread blocks in the grid
#     blockspergrid = (data.size + (threadsperblock - 1)) # threadsperblock

#     # Now start the kernel
#     my_kernel[blockspergrid, threadsperblock](data)

#     # Print the result
#     print(data)    

# if __name__=="__main__":

#     n = 10000000
#     a = np.ones(n, float)

#     b = cuda.device_array(n, float)

#     start = timer()

#     sum_a = func(a)
#     print(f"CPU {timer()-start:.2f}s\t{sum_a}")

#     start = timer()

#     func2(b)

#     sum_b = sum(b.copy_to_host())

#     # cuda.profile_stop()

#     print(f"GPU {timer()-start:.2f}s\t{sum_b}")
