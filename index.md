# Optimum Path Planning for Arbitrary Mazes

## Problem Definition

## Solution

### Path Optimization
Once the map has been encoded as a graph, the robot has to plan a path through
the maze.  There are many possible algorithms to do this, but here, breadth
first search (BFS) and aStar will be compared to see which algorithm gives the
best performance.  BFS is a complete algorithm, but aStar is both complete and
optimal.  Figure ___ shows animations of BFS and aStar.  
![image](/Presentation/bfs.gif)
![image](/Presentation/astar.gif)  
*Figure ___: Animation of BFS on left and aStar on right.*

### Line Following
In order to stay on the lines drawn in blue tape, the robot uses a set of five
reflectance sensors on its bottom.  It uses a type of p-control, where the robot
tries to drive so that the line is only under the middle reflectance sensor.  
The line following control is based on Richard T. Vannoy II's [“Design a Line Maze Solving Robot”](https://www.pololu.com/file/0J195/line-maze-algorithm.pdf).  Figure ___
shows the sensors used for line detection.  
![image](/Presentation/sensors.png)
![image](/Presentation/reflectance.png)  
*Figure ___: The loose reflectance sensors on the left, and applied to the robot
on the right.*

### Route Following
The other part of the robot's control is to follow the route given by the motion
planning algorithm.  At each step, the robot checks if it is at an intersection
using the reflectance sensors.  If so, then it executes the next step in the
directions for completing the maze, using the line following control between
intersections.  This algorithm is shown in Pseudo Code ---.  

```python
directions = [-1 turn left, 0 go straight, 1 turn right]

def line_follow_control(self):
	if at intersection:
		rotate according to directions
		move off intersection
	else:
		move forward staying on line
```  
*Pseudo Code ___: Route following algorithm*
## Results

### Success!  
<iframe width="560" height="315" src="https://drive.google.com/open?id=1pcBH6b0eSH8b0hWmrWPS9H6eVNmzyZ7X" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>  

### Statistics
Figure ___ shows a graph of the difference between the number of nodes expanded
for aStar and BFS against the number of nodes in the maze graph.  The data was
obtained by taking the mean and standard deviation of the number of nodes
expanded for each algorithm on 1000 randomly generated graphs.  
![image](/Presentation/NodeComparison.png)
*Figure ___: Difference between the number of nodes expanded for each algorithm*

Figure ___ shows a graph of the amount of time taken to arrive at a solution for
aStar and BFS against the number of nodes in each maze graph.  The data was
obtained by taking the mean and standard deviation of the number of nodes
expanded for each algorithm on 1000 randomly generated graphs.  
![image](/Presentation/AlgorithmSpeedComparison.png)  
*Figure ___: Time taken to find solution for each algorithm*

Figure ___ shows a graph comparing the path distance of the solution for aStar
and BFS.  The data was obtained by taking the mean of the path distance for each
algorithm on 1000 randomly generated graphs with 1000 nodes in them.  On average
aStar produced a path that was 1005 px or 29% shorter than BFS.  
![image](/Presentation/pathDist.png)  
*Figure ___: Path distance for solution of each algorithm*

## Conclusions

* Image processing can produce useful maps
* A* produces more optimal paths, and more efficiently, than BFS
* Difference in run time is negligible
* Motion planning time is negligible compared to travel time


## References

M. O. A. Aqel, A. Issa, M. Khdair, M. Elhabbash, M. Abubaker, and M. Massoud, “Intelligent Maze Solving Robot Based on Image Processing and Graph Theory Algorithms,” 2017 International Conference on Promising Electronic Technologies (ICPET), pp. 48–53, 2017.

R. O. Duda and P. E. Hart, “Use of the Hugh Transformation to Detect Lines and Curves in Pictures,” Comm. ACM, vol. 15, no. 1, pp. 11–15, Jan. 1972.

D. Ferguson, M. Likhachev, and A. Stentz, “A Guide to Heuristic-based Path Planning,” Carnegie Mellon School of Computer Science.

R. Paranjpe and A. Saied, “A Maze Solver for Android.” Stanford Digital Stacks.

Richard T. Vannoy II, “Design a Line Maze Solving Robot” https://www.pololu.com/file/0J195/line-maze-algorithm.pdf

Testing website.

![image](/maze_photos/cycles.jpg)

## Adding more things to see how they look

[link to repo](https://github.com/simonlimon/E160-Final-Project)

### More data

1. yes
2. this
3. works
