# Visibility graph in Unity [WIP]
Lee's visibility graph implementation in Unity

- Uses algorithm : http://cs.smith.edu/~streinu/Teaching/Courses/274/Spring98/Projects/Philip/fp/algVisibility.htm

- More details here : http://dav.ee/papers/Visibility_Graph_Algorithm.pdf

- Pathfinding uses Djikstra with adjacency list

- Works strictly on XZ plane

- Lots of functions are called with float x,y,z parameters and not with Vector3, because passing a struct value this many times really seems to cripple the performance (doesn't make any sense?)

- Using AVL tree from : https://code.google.com/archive/p/self-balancing-avl-tree/

- Polygon intersection is not handled (yet)
