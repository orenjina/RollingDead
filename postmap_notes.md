## General description of uses of postmap functions:

Def: Robot information includes robot position, direction, health, and energy.

# astar(graph, dimensions, start, end):
  Standard search algorithm using the manhattan distance as heuristics. Return some directions on how to reach there. Might make this return more results in the future (Return something for every fruit and order the results by fruit type and distance)

# fruit[int][int] double :
  An array containing each fruit type. The first index is based on the color of the fruit, by some deterministic way of organizing. The second index is based on the function of that color of the fruit. After accessing both indices, the data structure return the probability of that color of fruit resulting in that property.

# vectorSum(vectors):
  Simple vector sum auxiliary function.

# arbiter(fruit, zombies, robot information, map):
  Use the given information to determine what is the immediate next action. May use astar to find out which fruit to go for.
