
import numpy as np

RNG = np.random.default_rng()

def rouletteSelection(probs): 
    pick = RNG.random()*sum(probs) # stop the roulette in a random point
    current = 0.
    for x, prob in enumerate(probs): # walk on the roulette
        current += prob
        if current > pick:
            return x # return the edge index
    return -1

def run_simulation(num_rolls, probabilities):
  
    probs = [None for _ in probabilities]
    for side, prob in probabilities.items():
        probs[side-1] = prob

    # initialize the outcomes
    outcomes = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
    for _ in range(num_rolls):
        roll = rouletteSelection(probs)
        outcomes[roll+1] += 1
    return outcomes


if __name__ == "__main__":

    num_rolls = 10000 # number of rolls in a roulette

    probabilities = {1: 0.20,
                     2: 0.28,
                     3: 0.08,
                     4: 0.16,
                     5: 0.24,
                     6: 0.04}

    outcomes = run_simulation(num_rolls, probabilities)

    for outcome, count in outcomes.items():
        print(f"Outcome {outcome}: {count} ({count / num_rolls * 100:.2f}%)")
