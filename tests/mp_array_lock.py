import multiprocessing as mp
import numpy as np
import ctypes as c

#https://superfastpython.com/multiprocessing-shared-ctypes-in-python/

n = 2
m = 3

def addData(shared_array, shape, lock, process_number):

    array = to_numpy_array(shared_array, shape)
    n,m = shape
    i=0
    for nn in range(n):
        for mm in range(m):
            with lock:
                array[nn][mm] += i
            i=i+1

    print(f"Array after process {process_number}")
    print(array, flush=True)

def to_shared_array(arr, ctype):
    shared_array = mp.Array(ctype, arr.size, lock=False)
    temp = np.frombuffer(shared_array, dtype=arr.dtype)
    temp[:] = arr.flatten(order='C')
    return shared_array

def to_numpy_array(shared_array, shape):
    '''Create a numpy array backed by a shared memory Array.'''
    arr = np.ctypeslib.as_array(shared_array)
    return arr.reshape(shape)

if __name__=='__main__':

    # Start with a numpy array!
    mp_arr = np.zeros((n, m), dtype=np.int32)
    shared_array = to_shared_array(mp_arr, c.c_int32)

    # you have to now use the shared array as the base
    u = to_numpy_array(shared_array, mp_arr.shape)

    print("Array at the inital state: ")
    print(u)

    lock = mp.Lock()

    p1=mp.Process(target=addData,args=(shared_array, mp_arr.shape, lock, 1))
    p2=mp.Process(target=addData,args=(shared_array, mp_arr.shape, lock, 2))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

    print("Array at the final state: ")
    print(u)