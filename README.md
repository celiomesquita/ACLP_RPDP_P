# Air cargo load and route planning in pickup and delivery operations

In the aerial pickup and delivery of goods in a distribution network, transport aviation faces risks of cargo unbalancing due to the urgency required for loading, immediate take-off, and mission accomplishment. It is especially critical for disaster relief, short deadlines, or any pressure for immediate takeoff. In addition, there are no commercially available systems that can assist load and trip planners with pallet building and aircraft-balanced loading plans, with demands for transport at each hub. This enables other risks, such as improper delivery, excessive fuel burn, and a longer than necessary turn-around time.

We defined and solved the problem of planning the loading and routing of a single aircraft according to a utility score, weight and balance principles, and fuel consumption in a tour of simultaneous pickup and delivery.

This NP-hard problem, named *Air Cargo Load Planning with Routing, Pickup, and Delivery Problem* (ACLP+RPDP), is mathematically modeled using standardized pallets in fixed positions, obeying center of gravity constraints, delivering each item to its destination, and minimizing fuel consumption costs. We performed multiple experiments with a commercial solver and four well-known meta-heuristics on data based on the transport history of the *Brazilian Air Force*.

We also designed a new heuristic that quickly finds practical solutions to a wide range of problem sizes, a key contribution that resolved all real scenarios in much less time than is operationally acceptable.

Our main contributions were the mathematical model of ACLP+RPDP that involves four *NP-hard* sub-problems, a complete process to solve it on a simple handheld computer, and a new heuristic that offers fast node solutions with good quality.

Finally, by focusing on the node-to-node solution, the method of this work is not exclusive to aircraft and airports; it can be adapted, for example, to ships and ports, vehicles and warehouses, or wagons and railways, provided that their practical cases are similar to those considered here. In these situations, it would be necessary to make some changes in the model: for example, modify the load balancing constraints and consider the available space in vehicles or wagons instead of pallets.

As this is ongoing research, we thought about some possible future improvements:

1. consider an aircraft fleet rather than a single one;
2. model three-dimensional items;
3. discard the highest-cost tours to reduce the runtime;
4. implement parallel algorithms in some steps of the solution.

*Preprint submitted to Expert Systems with Applications*

## Execution instructions

### Before testing, it is necessary to have a complete understanding of the paper's content.

*Mesquita, A.C.P., Sanches, C.A.A., 2023. Air cargo load and route planning in pickup and delivery operations.
Expert Systems with Applications, XX–XX.*

1. Open the main.py file
2. Choose a time limit for the solver to work on a node.
3. Choose which scenarios you are going to run:
    2 -> scenario 1, 3 -> scenario 2, and so on.
4. Choose the volume surplus> 1.2, 1.5, or 2.0
5. Choose a solution method
6. Run the file.
7. Check the results in folder **results**.
