import numpy as np

def greedy_heuristic(items, weights, values, capacity):
  """
  Greedy algorithm to select items with highest value-to-weight ratio.
  """
  sorted_items = sorted(zip(items, weights, values), key=lambda x: x[2]/x[1], reverse=True)
  selected_items = []
  total_weight = 0
  for item, weight, value in sorted_items:
    if total_weight + weight <= capacity:
      selected_items.append(item)
      total_weight += weight
  return selected_items

def lagrangian_relaxation(items, weights, values, capacity, lagrange_multiplier):
  """
  Lagrangian relaxation to estimate near-optimal solution.
  """
  n = len(items)
  dual_values = np.zeros(n)
  for i in range(n):
    dual_values[i] = values[i] - lagrange_multiplier * weights[i]
  sorted_items = sorted(zip(items, weights, dual_values), key=lambda x: x[2], reverse=True)
  selected_items = []
  total_weight = 0
  for item, weight, dual_value in sorted_items:
    if total_weight + weight <= capacity:
      selected_items.append(item)
      total_weight += weight
  return selected_items

def hybrid_approach(items, weights, values, capacity):
  """
  Hybrid approach combining greedy and Lagrangian relaxation.
  """
  lagrange_multiplier = 0
  while True:
    # Greedy step
    greedy_items = greedy_heuristic(items, weights, values, capacity)
    greedy_weight = sum(weights[i] for i in greedy_items)

    # Lagrangian relaxation step
    relaxed_items = lagrangian_relaxation(items, weights, values, capacity, lagrange_multiplier)
    relaxed_weight = sum(weights[i] for i in relaxed_items)

    # Update Lagrange multiplier
    if greedy_weight < capacity and relaxed_weight > capacity:
      lagrange_multiplier += 0.1  # Adjust step size based on your needs
    else:
      break

  # Final selection based on relaxed solution
  return relaxed_items



if __name__ == "__main__":

    # Example usage (replace with your actual data)
    items = ["A", "B", "C", ...]
    weights = [10, 20, 30, ...]
    values = [50, 60, 70, ...]
    capacity = 100    
  
    selected_items = hybrid_approach(items, weights, values, capacity)
    print(f"Selected items: {selected_items}")
