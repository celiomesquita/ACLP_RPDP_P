from time import time


# Python3 code for Dynamic Programming
def Solve(capacity, weights, profits):

    n = len(profits)

    K = [[0 for _ in range(capacity + 1)] for _ in range(n + 1)]
			
	# Build table K[][] in bottom up manner
    for i in range(n + 1):
        for w in range(capacity + 1):
            if i == 0 or w == 0:
                K[i][w] = 0
            elif weights[i - 1] <= w:
                K[i][w] = max(profits[i - 1] + K[i - 1][w - weights[i - 1]],	K[i - 1][w])
            else:
                K[i][w] = K[i - 1][w]


    solution = []

	# stores the result of Knapsack
    res = K[n][capacity]
    w   = capacity

    for i in range(n, 0, -1):
        if res <= 0:
            break
		# either the result comes from the top (K[i-1][w]) or from
        #  (profits[i-1] + K[i-1] [w-weights[i-1]]) as in Knapsack table.
        # If it comes from the latter one it means the item is included.
        if res == K[i - 1][w]:
            continue
        else:

            solution.append(i-1)
			
			# Since this weights is included its value is deducted
            res -= profits[i-1]
            w   -=  weights[i-1]    

    return solution

# Driver program to test above function
if __name__=='__main__':
	
    weights  = [70, 73, 77, 80, 82, 87, 90, 94, 98,106,110,113,115,118,120]
    profits  = [135,139,149,150,156,163,173,184,192,201,210,214,221,229,240]
    capacity = 750

    start = time()

    solution = Solve(capacity, weights, profits) 

    elapsed = time() - start
    elapsed *= 1000
    
    print(f"{elapsed:.1f}ms")

    value = 0
    weight = 0
    for s in solution:
        value  += profits[s]
        weight += weights[s]

    print(solution, value, weight)
