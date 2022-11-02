import multiprocessing
import time
from multiprocessing import Pool
from joblib import Parallel, delayed
from functools import partial

def f1(a):
    c = 0
    for i in range(0, 99999999):
        c = c + 1
    return 1


# def f2(b):
#     c = 0
#     for i in range(0, 99999999):
#         c = c + 1
#     return 1

if __name__ == '__main__':

    t0 = time.time()
    aa = f1(1)
    bb = f1(2)
    print(f"serial\t{time.time()-t0:.2f}")    

    num_cores = multiprocessing.cpu_count()

    pool = Pool(num_cores)
    result1 = pool.apply_async(f1, [1])
    result2 = pool.apply_async(f1, [2])

    t0 = time.time()
    answer1 = result1.get(timeout=10)
    answer2 = result2.get(timeout=10)
    print(f"pool\t{time.time()-t0:.2f}")


    t0 = time.time()
    input = [1,2] 
    cc = partial(f1)
    output = Parallel(n_jobs=num_cores)(delayed(cc)(i) for i in input)
    print(f"joblib\t{time.time()-t0:.2f}")   