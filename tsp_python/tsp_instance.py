
from brkga_mp_ipr.exceptions import LoadError

class TSPInstance():

    def __init__(self, filename: str):

        with open(filename, "r") as hd:
            lines = hd.readlines()

        if not lines:
            raise LoadError(f"Cannot read file '{filename}'")

        line_number = 1
        try:
            self.num_nodes = int(lines[0])

            self.distances = []

            for i in range(1, self.num_nodes):
                line_number = i + 1
                values = [float(x.strip()) for x in lines[i].split()]
                self.distances.extend(values)
        except Exception:
            raise LoadError(f"Error reading line {line_number} of '{filename}'")

    def distance(self, i: int, j: int) -> float:
        if i > j :
            i, j = j, i
        coord = (i * (self.num_nodes - 1)) - ((i - 1) * i // 2) + (j - i - 1)
        if coord < len(self.distances):
            return self.distances[coord]
        else:
            return 0

