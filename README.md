# Visibility graph in Unity [WIP]
de Berg's visibility graph implementation in Unity

- Uses algorithm : http://cs.smith.edu/~streinu/Teaching/Courses/274/Spring98/Projects/Philip/fp/algVisibility.htm

- Pathfinding uses Djikstra with adjacency list

- Works strictly XZ plane

- Lots of functions are called with float x,y,z parameters and not with Vector3, because passing a struct value this many times really seems to cripple the performance

- Using AVL tree from : https://code.google.com/archive/p/self-balancing-avl-tree/
