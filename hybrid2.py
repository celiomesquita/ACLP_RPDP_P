def hybrid_knapsack(items, capacity, weight_limit, value_function):
  """
  Solves a 0/1 knapsack problem with a hybrid approach.

  Args:
    items: A list of items, where each item is a dictionary with keys
      "weight" and "value".
    capacity: The maximum weight allowed in the knapsack.
    weight_limit: An additional weight constraint (optional).
    value_function: A function that calculates the value of a subset of items.

  Returns:
    A tuple containing the selected items and their total value.
  """

  # Phase 1: Greedy Heuristic
  selected_items = []
  remaining_capacity = capacity
  remaining_weight_limit = weight_limit

  while remaining_capacity > 0 and remaining_weight_limit > 0:
    best_item = None
    best_value_per_unit = 0
    for item in items:
      if item not in selected_items:
        if item["weight"] <= remaining_capacity and \
           item["weight"] <= remaining_weight_limit:
          value_per_unit = value_function(selected_items + [item]) / item["weight"]
          if value_per_unit > best_value_per_unit:
            best_item = item
            best_value_per_unit = value_per_unit

    if best_item:
      selected_items.append(best_item)
      remaining_capacity -= best_item["weight"]
      remaining_weight_limit -= best_item["weight"]

  # Phase 2: Lagrangian Relaxation
  # (Implementation details omitted for brevity)

  # Phase 3: Combine solutions and refine (optional)
  # (Implementation details omitted for brevity)

  return selected_items, value_function(selected_items)

# Example usage:
items = [
  {"weight": 5, "value": 10},
  {"weight": 3, "value": 7},
  {"weight": 2, "value": 5},
  {"weight": 7, "value": 15},
]
capacity = 10
weight_limit = 8

selected_items, total_value = hybrid_knapsack(items, capacity, weight_limit, sum)

print("Selected items:", selected_items)
print("Total value:", total_value)
