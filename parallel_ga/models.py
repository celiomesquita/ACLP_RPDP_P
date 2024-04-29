class Item:
    def __init__(self, id, weight, volume, value, dest):
        self.id = id
        self.weight = weight
        self.volume = volume
        self.value  = value
        self.dest = dest

    def __repr__(self):
        return f"Item(ID: {self.id}, Weight: {self.weight}, Volume: {self.volume}, Value: {self.value})"

class Knapsack:
    def __init__(self, dist_cg, id, max_weight, max_volume, dest ):
        self.id = id
        self.max_weight = max_weight
        self.max_volume = max_volume
        self.dist_cg    = dist_cg
        self.dest       = dest
        # variables
        self.items = []
        self.current_weight = 0.
        self.current_volume = 0.
        self.penalty        = 0.

    def can_add_item(self, item, torque, itemsDict_ga):
        can_add =itemsDict_ga["mpItems"][item.id] == 0 and self.dest == item.dest
        
        # (self.current_volume + item.volume <= self.max_volume) and\
        # (self.current_weight + item.weight <= self.max_weight) and\
        # (abs(torque["current"] + self.dist_cg * item.weight) <= torque["maximum"]) and\

        return can_add

    def add_item(self, item, torque, solDict_ga, itemsDict_ga, N):
        
        if self.can_add_item(item, torque, itemsDict_ga):
            
            self.items.append(item)
            self.current_weight += item.weight
            self.current_volume += item.volume

            torque["current"] += self.dist_cg * item.weight
            solDict_ga["solMatrix"][N*self.id + item.id] = 1 # solution array
            itemsDict_ga["mpItems"][item.id] = 1 # to control items inclusion

            if self.current_volume  > self.max_volume:
                self.penalty += item.value

            if self.current_weight  > self.max_weight:
                self.penalty += item.value

            if torque["current"]  > torque["maximum"]:
                self.penalty += item.value

            return True
        return False

    def try_add_item(self, item, torque, solDict_ga, itemsDict_ga, N):
        """Attempts to add an item to the knapsack if it fits."""
        if self.can_add_item(item, torque, itemsDict_ga):
            self.add_item(item, torque, solDict_ga, itemsDict_ga, N)
            return True
        return False
    
    def total_value(self):
        return sum(item.value for item in self.items) - self.penalty

    def __repr__(self):
        return f"Knapsack(ID: {self.id}, Items: {len(self.items)}, Total Value: {self.total_value()})"
    
    def reset(self):
        """Clears the knapsack's contents and resets its current weight and volume."""
        self.items = []
        self.current_weight = 0.
        self.current_volume = 0.
        self.penalty = 0.


    def print_results(self):
        print(f"{self.id}\t{self.current_weight:.0f}\t{self.current_volume:.2f}\t{len(self.items)}")