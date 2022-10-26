import timeit

# the file name must different from the import module name

def fibonacci(n):
    """
    Returns the n-th Fibonacci number.
    """
    if(n == 0):
        result = 0
    elif(n == 1):
        result = 1
    else:
        result = fibonacci(n-1) + fibonacci(n-2)
    return result

if __name__ == '__main__':

    t1 = timeit.Timer("fibonacci(13)", "from __main__ import fibonacci")

    print(f"fibonacci ran: {t1.timeit(number=1000):.4f} ms")

    code = """   
limit = 10000
prime_list = [i for i in range(2, limit+1)]

for prime in prime_list:
    for elem in range(prime*2, max(prime_list)+1, prime):
        if elem in prime_list:
            prime_list.remove(elem)
"""    

    print(f"code time: {timeit.timeit(code, number=10):.1f} s")    