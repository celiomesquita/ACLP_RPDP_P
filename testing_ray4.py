#!/usr/bin/env python3
import time
import ray

@ray.remote
def add(x, y):
    time.sleep(1)
    return x + y

# Aggregate the values slowly. This approach takes O(n) where n is the
# number of values being aggregated. In this case, 7 seconds.
id1 = add.remote(1, 2)
id2 = add.remote(id1, 3)
id3 = add.remote(id2, 4)
id4 = add.remote(id3, 5)
id5 = add.remote(id4, 6)
id6 = add.remote(id5, 7)
id7 = add.remote(id6, 8)

start_time = time.time()
result = ray.get(id7)
duration = time.time() - start_time

print(f"O(n): {duration:.3f}")

# Aggregate the values in a tree-structured pattern. This approach
# takes O(log(n)). In this case, 3 seconds.
id1 = add.remote(1, 2)
id2 = add.remote(3, 4)
id3 = add.remote(5, 6)
id4 = add.remote(7, 8)
id5 = add.remote(id1, id2)
id6 = add.remote(id3, id4)
id7 = add.remote(id5, id6)

start_time = time.time()
result = ray.get(id7)
duration = time.time() - start_time
print(f"O(log(n)): {duration:.3f}")


# # Slow approach.
# values = [1, 2, 3, 4, 5, 6, 7, 8]
# while len(values) > 1:
#     values = [add.remote(values[0], values[1])] + values[2:]
# result = ray.get(values[0])


# # Fast approach.
# values = [1, 2, 3, 4, 5, 6, 7, 8]
# while len(values) > 1:
#     values = values[2:] + [add.remote(values[0], values[1])]
# result = ray.get(values[0])