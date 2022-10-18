#!/usr/bin/env python3
import time
import ray
import numpy as np

@ray.remote
def create_matrix(size):
    return np.random.normal(size=size)

@ray.remote
def multiply_matrices(x, y):
    return np.dot(x, y)

x_id = create_matrix.remote([1000, 1000])
y_id = create_matrix.remote([1000, 1000])
z_id = multiply_matrices.remote(x_id, y_id)

start_time = time.time()

# Get the results.
z = ray.get(z_id)


duration = time.time() - start_time

print(f"{z[:10]} {duration:.2f}")