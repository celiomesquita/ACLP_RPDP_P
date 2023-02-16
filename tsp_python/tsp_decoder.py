
from brkga_mp_ipr.types import BaseChromosome
from tsp_instance import TSPInstance

class TSPDecoder():

    def __init__(self, instance: TSPInstance):
        self.instance = instance

    def decode(self, chromosome: BaseChromosome, rewrite: bool) -> float:

        permutation = sorted(
            (key, index) for index, key in enumerate(chromosome)
        )

        cost = self.instance.distance(permutation[0][1], permutation[-1][1])

        for i in range(len(permutation) - 1):
            cost += self.instance.distance(permutation[i][1], permutation[i + 1][1])
        
        return cost
