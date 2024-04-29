from random import random
from time import sleep
from multiprocessing.pool import Pool
 

def task(test_dict, identifier, value):

    test_dict["age"] = int(100 * value)

    print(f'Task {identifier} executing with {value:.2f} name:{test_dict["name"]} age:{test_dict["age"]}', flush=True)

    sleep(value)

    return identifier, value, test_dict
 

if __name__ == '__main__':

    test_dict = {"name":"John","age":44}

    with Pool() as pool:

        items = [(test_dict, i, random()) for i in range(10)]

        for result in pool.starmap(task, items):
            print(f'Got result: {result}', flush=True)

