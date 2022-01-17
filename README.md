# BiDirectionalWeightedAstar
Bi-directional weighted A* planning algorithm written in C++ using multi-threading features, faster than A* for certain goal locations in a map
 Implements weighted bidirectional Astar path planning algorithm, using two threads one from source and the other from the destination
 main function takes in 5 parameters 
 1. x coordinate of start point
 2. y coordinate of start point
 3. x coordinate of goal point
 4. y coordinate of goal point
 5. w, weighted for the weighted Astar
 
 Compiled using c++11
 ``` 
 $ g++ -std=c++11 -pthread  main.cpp -o x 
 $ ./x 1 1 300 300 1.1 
  ```
Using cmake
```
$ mkdir build
$ cd build/
$ cmake ../
$ make
$ ./result 
```
