# BelkanGame
## Miguel García López
### Practice - Artificial Intelligence

- Description: Small game that implements several basic Pathfinding algorithms, such as Depth Search, Breadth Search etc. It has several levels:
  - Lv 0 -> demo level, without any difficulty with basic depth search pathfinding algorithm without any consideration of objects, obstacles or other considerations.
  - Lv 1 -> an optimal plan in number of actions is required. For this level, the breadth search algorithm is used.
  - Lv 2 -> an optimal plan is required regarding the spent battery. To do this, you have to take into account the cost of each square and different objects that recover it. A uniform cost algorithm is used.
  - Lv 3 -> same as level 2, but with the added bonus of finding three targets instead of one. Uniform cost is still used.
  - Lv 4 -> find the maximum number of targets possible in an unexplored map (we don't have information about the squares until we explore them).

- To run it:
  - ./install.sh to install necessary packages
  - make to compile
  - ./Belkan to run the program
