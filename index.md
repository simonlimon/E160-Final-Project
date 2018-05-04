## Problem Definition

#### Given an *arbitrary physical maze*, drawn with blue tape on a white surface, find the *shortest path* from the origin to the goal. Program a [Pololu Romi](https://www.pololu.com/category/203/romi-chassis-kits) robot to *follow this path*.

Figure CH.1 shows the problem at hand decomposed in four progressive steps, listing the technologies used for each of them. 

![image](/Presentation/steps.png)

*Figure CH.1: Animation of BFS on left and A\* on right.*

## Solution

### Maze Encoding

The first step consists of encoding the maze as a graph in order to be able to perform a path search through it using traditional graph algorithms. We decided to build a graph where each node represents an intersection, and each edge represents a connection between the intersections. This encoding simplifies the process of generating turn-by-turn instructions for the robot to reach the goal. 

In order to build this graph we used various image processing techniques from the Python [scikit-image](http://scikit-image.org/docs/dev/api/skimage.html) library. Consider the example image of a maze shown in Figure ME.1. The first step we took was to skeletonize this image, as show in Figure ME.2. We then tried unsuccessfully to construct a graph by finding the lines in the image through a Hough transform and calculating the intersections geometrically, but there were too many lines detected by the transform as shown in Figure ME.3. Our second approach was to use a corner detection method, shown in Figure ME.4, but we then struggled when trying to connect the detected corners correctly in a graph. 

![image](/Presentation/raw.jpg) ![image](/Presentation/skel.png)

*Figures ME.1 and ME.2: Raw and skeletonized versions of the maze.*

![image](/Presentation/lines.png) ![image](/Presentation/corner.png)

*Figures ME.3 and ME.4: Hough lines transform and corner detection.*

Finally, we were able to correctly construct the graph by writing an algorithm to recursively traverse the skeletonized image following the white pixels. The pseudocode for this procedure is show in Pseudo Code ME.1. And the results can be observed in Figure ME.4

```python
def traverse(G, img, box, parent_node):
    while box is on top of line and not intersection:
            move box forward

        new_node = node at box location
        add new_node to graph
        new_boxes = replicate depending on intersection type

        for box in new_bozes:
            traverse(G,img,box,new_node)
```

*Pseudo Code ME.1: Graph building algorithm*

![image](/Presentation/sol.png)

*Figure ME.5: Output of traversal algorithm.*

### Path Optimization
Once the map has been encoded as a graph, the robot has to plan a path through
the maze.  There are many possible algorithms to do this, but here, breadth
first search (BFS) and A\* will be compared to see which algorithm gives the
best performance.  BFS is a complete algorithm, but A\* is both complete and
optimal in terms of shortest distance given weighted edges.  Figure PO.1 shows animations of BFS and A\*.  

![image](/Presentation/bfs.gif)
![image](/Presentation/astar.gif)  
*Figure PO.1: Animation of BFS on left and aStar on right.*

### Line Following
In order to stay on the lines drawn in blue tape, the robot uses a set of five
reflectance sensors on its bottom.  It uses a type of p-control, where the robot
tries to drive so that the line is only under the middle reflectance sensor.  
The line following control is based on Richard T. Vannoy II's [“Design a Line Maze Solving Robot”](https://www.pololu.com/file/0J195/line-maze-algorithm.pdf).  Figure LF.1
shows the sensors used for line detection.  

![image](/Presentation/sensors.png)
![image](/Presentation/reflectance.png)  

*Figure LF.1: The loose reflectance sensors on the left, and applied to the robot on the right.*

### Route Following
The other part of the robot's control is to follow the route given by the motion
planning algorithm.  At each step, the robot checks if it is at an intersection
using the reflectance sensors.  If so, then it executes the next step in the
directions for completing the maze, using the line following control between
intersections.  This algorithm is shown in Pseudo Code RF.1.  

```python
directions = [-1 turn left, 0 go straight, 1 turn right]

def line_follow_control(self):
	if at intersection:
		rotate according to directions
		move off intersection
	else:
		move forward staying on line
```
*Pseudo Code RF.1: Route following algorithm*
## Results

### Success!  
<iframe src="https://drive.google.com/file/d/1pcBH6b0eSH8b0hWmrWPS9H6eVNmzyZ7X/preview" width="640" height="480"></iframe>
If the video won't load, try [this link.](https://drive.google.com/open?id=1pcBH6b0eSH8b0hWmrWPS9H6eVNmzyZ7X)  
### Statistics
Figure S.1 shows a graph of the difference between the number of nodes expanded
for aStar and BFS against the number of nodes in the maze graph.  The data was
obtained by taking the mean and standard deviation of the number of nodes
expanded for each algorithm on 1000 randomly generated graphs.  

![image](/Presentation/NodeComparison.png)
*Figure S.1: Difference between the number of nodes expanded for each algorithm*

Figure S.2 shows a graph of the amount of time taken to arrive at a solution for
aStar and BFS against the number of nodes in each maze graph.  The data was
obtained by taking the mean and standard deviation of the number of nodes
expanded for each algorithm on 1000 randomly generated graphs.  

![image](/Presentation/AlgorithmSpeedComparison.png)  
*Figure S.2: Time taken to find solution for each algorithm*

Figure S.3 shows a graph comparing the path distance of the solution for aStar
and BFS.  The data was obtained by taking the mean of the path distance for each
algorithm on 1000 randomly generated graphs with 1000 nodes in them.  On average
aStar produced a path that was 1005 px or 29% shorter than BFS.  

![image](/Presentation/pathDist.png)  
*Figure S.3: Path distance for solution of each algorithm*

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

