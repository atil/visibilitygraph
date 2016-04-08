# Visibility graph in Unity [WIP]
de Berg's visibility graph implementation in Unity

- Used algorithm explained in the following link: 
http://cs.smith.edu/~streinu/Teaching/Courses/274/Spring98/Projects/Philip/fp/algVisibility.htm

- Uses strictly XZ plane

- Polygons are assumed to be convex (for the moment)

- Lots of functions are called with float x,y,z parameters and not with Vector3, because passing a struct value this many times really seems to cripple performance
