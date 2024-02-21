class Item:
    def __init__(self, item_id, weight, volume, value):
        self.item_id = item_id
        self.weight = weight
        self.volume = volume
        self.value = value

    def __repr__(self):
        return f"Item(ID: {self.item_id}, Weight: {self.weight}, Volume: {self.volume}, Value: {self.value})"

class Knapsack:
    def __init__(self, knapsack_id, max_weight, max_volume):
        self.knapsack_id = knapsack_id
        self.max_weight = max_weight
        self.max_volume = max_volume
        self.items = []
        self.current_weight = 0
        self.current_volume = 0

    def can_add_item(self, item):
        return (self.current_weight + item.weight <= self.max_weight) and (self.current_volume + item.volume <= self.max_volume)

    def add_item(self, item):
        if self.can_add_item(item):
            self.items.append(item)
            self.current_weight += item.weight
            self.current_volume += item.volume
            return True
        return False

    def total_value(self):
        return sum(item.value for item in self.items)

    def __repr__(self):
        return f"Knapsack(ID: {self.knapsack_id}, Items: {len(self.items)}, Total Value: {self.total_value()})"
    
    def reset(self):
        """Clears the knapsack's contents and resets its current weight and volume."""
        self.items = []
        self.current_weight = 0
        self.current_volume = 0

    def try_add_item(self, item):
        """Attempts to add an item to the knapsack if it fits."""
        if self.can_add_item(item):
            self.add_item(item)
            return True
        return False

    def print_results(self):
        print(f"{self.knapsack_id}\t{self.current_weight:.0f}\t{self.current_volume:.2f}\t{len(self.items)}")
