import networkx as nx
import math
import time
from Node import *
#import matplotlib.pyplot as plt
from motionPlanning import *
#from E160_wall import *

def directions(walls, start, end, isAStar):
    wallLines = [[round(x, 1) for x in wall.simple_points] for wall in walls]
    G = graphFromLines(wallLines)

    startNode = Node(start[0], start[1], str(start))
    endNode = Node(end[0], end[1], str(end))
    breadthPath = breadthFirstPath(G, startNode, endNode)
    aPath = aStarPath(G, startNode, endNode)

    if isAStar:
        return pathToDirections(aPath)
    else:
        return pathToDirections(breadthPath)

# Takes in an array of lines and returns a networkx graph with nodes at the
# intersection points and edges with attribute 'length' equal to the distances
# between neighboring
def graphFromLines(lines):
    intersects = findAllIntersects(lines)

    G = nx.Graph()

    for line in lines:
        inters = intersects[str(line)]

        end1 = Node(line[0], line[1], str(line[0:2]))
        end2 = Node(line[2], line[3], str(line[2:]))

        # Add the line end points to the graph if they aren't already there
        if end1 not in G:
            G.add_node(end1)
        if end2 not in G:
            G.add_node(end2)

        # Add all of the intersections on this line to the graph if they
        # aren't already there
        for i in range(len(inters)):
            inter = inters[i]

            node = Node(inter[0], inter[1], str(inter))

            if node not in G:
                G.add_node(node)

            # Link the intersections to their neighbors with edges
            if i == 0:
                G.add_edge(end1, node, length=distance(end1, node))
            else:
                oldInt = inters[i-1]
                neighbor = Node(oldInt[0], oldInt[1], str(oldInt))

                G.add_edge(neighbor, node, length=distance(neighbor, node))

            if i == len(inters) - 1:
                G.add_edge(end2, node, length=distance(end2, node))

            pass

    return G


# Takes in an array of lines and returns a dictionary of the intersections
# that exist on each line.  A particular intersection will be located on both
# lines that intersect in the dictionary.  All of the intersections on a line
# will be sorted from closest to the first endpoint to farthest from the first
# endpoint.
def findAllIntersects(lines):
    intersects = {}

    for line1 in lines:
        for line2 in lines:
            if line1 != line2:
                inter = findIntersect(line1, line2)

                if inter != [float('inf'), float('inf')]:
                    # Sort the intersections properly
                    points = [inter]
                    if str(line1) in intersects:
                        points += intersects[str(line1)]

                    points = sortPoints(line1[0:2], points)

                    intersects[str(line1)] = points

    return intersects

# Sorts the points in the points list from closest to farthest from the
# reference point
def sortPoints(ref, points):
    # Find all of the distances from the reference point
    dists = [math.sqrt((point[0]-ref[0])**2 + (point[1]-ref[1])**2) for point in points]

    # Sort based on the distances
    return [point for _,point in sorted(zip(dists,points))]


# Takes two lines as inputs and finds the coordinates of the intersectionself.
# If there is no intersection it returns (inf, inf)
# The lines should be [x1,y1,x2,y2] where (x1,y1) and (x2,y2) are the two end
# points.
def findIntersect(line1, line2):

    # Assign shorter, more readable names
    x1_1, y1_1, x1_2, y1_2 = [float(x) for x in line1]
    x2_1, y2_1, x2_2, y2_2 = [float(x) for x in line2]

    # Ignore differences due to errors in numerical representation
    eps = 0.0001

    # l = left, r = right
    xl1 = min(x1_1, x1_2) - eps
    xr1 = max(x1_1, x1_2) + eps
    xl2 = min(x2_1, x2_2) - eps
    xr2 = max(x2_1, x2_2) + eps

    # b = bottom, t = top
    yb1 = min(y1_1, y1_2) - eps
    yt1 = max(y1_1, y1_2) + eps
    yb2 = min(y2_1, y2_2) - eps
    yt2 = max(y2_1, y2_2) + eps

    # Calculate slope
    m1 = 0.0
    m2 = 0.0
    if x1_1 != x1_2:
        m1 = (y1_1-y1_2)/float(x1_1-x1_2)
    else:
        m1 = float('inf')

    if x2_1 != x2_2:
        m2 = (y2_1-y2_2)/float(x2_1-x2_2)
    else:
        m2 = float('inf')

    # If the lines are parallel they don't intersect
    if m1 == m2:
        return [float('inf'), float('inf')]

    # Find the intercept of the two lines if they were infinite
    x_int = -(-m1*x1_1 + m2*x2_1 + y1_1 - y2_1)/float(m1 - m2)
    y_int = -((-m1*m2*x1_1 + m1*m2*x2_1 + m2*y1_1 - m1*y2_1)/float(m1 - m2))

    # If we have a vertical line then that didn't work
    if m1 == float('inf'):
        x_int = x1_1
        y_int = y2_2 - m2*float(x2_2 - x1_1)

    elif m2 == float('inf'):
        x_int = x2_1
        y_int = y1_2 - m1*float(x1_2 - x2_1)


    # Check if the intercept is on the segments
    if (xl1 <= x_int <= xr1) and (xl2 <= x_int <= xr2):
        if (yb1 <= y_int <= yt1) and (yb2 <= y_int <= yt2):
            return [x_int, y_int]

    return [float('inf'), float('inf')]

# Visualize a given graph
def drawGraph(G):

    options = {'node_color': 'red',
                'node_size': 100,
                'width': 3,
                'with_labels': True}

    plt.figure(6)
    nx.draw_kamada_kawai(G, **options)
    plt.title('kk')

    plt.show()

# Get some interesting statistics comparing breadth first search to aStar search
# in 1000 random graphs with 1000 nodes each.  The start and end nodes are the
# same for each algorithm on the same graph, but are randomly selected when the
# graphs are generated
def statistics():
    numGraphs = 50

    breadthDists = [float('inf')]*numGraphs
    aStarDists = [float('inf')]*numGraphs
    breadthNodes = [float('inf')]*numGraphs
    aStarNodes = [float('inf')]*numGraphs
    breadthTimes = [float('inf')]*numGraphs
    aStarTimes = [float('inf')]*numGraphs

    for i in range(numGraphs):
        print(i)
        G, startNode, endNode = randGraphMaze(10000, True)

        startTime = time.time()
        breadthPath, breadthNodes[i] = breadthFirstPathS(G, startNode, endNode)
        breadthTimes[i] = time.time()-startTime
        breadthDist = pathDistance(G, breadthPath)

        startTime = time.time()
        aPath, aStarNodes[i] = aStarPathS(G, startNode, endNode)
        aStarTimes[i] = time.time()-startTime
        aStarDist = pathDistance(G, aPath)

        breadthDists[i] = breadthDist
        aStarDists[i] = aStarDist


    avBreadthDist = sum(breadthDists)/len(breadthDists)
    avAStarDist = sum(aStarDists)/len(aStarDists)
    avBreadthNodes = sum(breadthNodes)/len(breadthNodes)
    avAStarNodes = sum(aStarNodes)/len(aStarNodes)
    avBreadthTime = sum(breadthTimes)/len(breadthTimes)
    avAStarTime = sum(aStarTimes)/len(aStarTimes)

    return avBreadthDist, avAStarDist, avBreadthNodes, avAStarNodes, avBreadthTime, avAStarTime


# Calculate the distance between two nodes
def distance(nodeA, nodeB):
    return ((nodeA.x-nodeB.x)**2 + (nodeA.y-nodeB.y)**2)**0.5

if __name__ == '__main__':
    print('*'*30, 'Test findIntersect', '*'*30)
    print('exp     0,0: ', findIntersect([0,0,1,0], [0,1,0,-1]))
    print('exp   0.5,0: ', findIntersect([0,0,1,0], [0.5,1,0.5,-1]))
    print('exp inf,inf: ', findIntersect([0,0,1,0], [-1,1,-1,-1]))
    print('exp 0.5,0.5: ', findIntersect([0,0,1,1], [0.5,1,0.5,-1]))
    print('exp inf,inf: ', findIntersect([0,0,1,0], [0,1,1,1]))
    print('exp 0.5,0.5: ', findIntersect([0,0,1,1], [0,1,1,0]))
    print('exp inf,inf: ', findIntersect([0,0,1,1], [0.707,0.6,0,-1]))
    print(' ')

    print('*'*28, 'Test findAllIntersects', '*'*28)
    lines = [[7,10,7,0], [0,0,10,0], [2,2,2,-2], [2,1,7,1],\
            [7,6,15,6], [7,4,15,4], [10,2,10,0], [10,1,15,1] ,[15,6,15,-2]]
    expInts = [[2,0], [8,0], [10,0], [10,2], [10,1], [15,1], [2,1], [7,1],\
            [7,4], [7,6], [15,6], [15,4]]
    foundInts = sum(list(findAllIntersects(lines).values()), [])
    all = True
    real = True

    # Check if it found all of the intersections
    for inter in expInts:
        all = (inter in foundInts) and all

        if inter not in foundInts:
            print('Did not find', int)

    # Check if it only found expected intersections
    for inter in foundInts:
        real = (inter in expInts) and real

        if inter not in expInts:
            print('Oddly, found', int)

    print('Found all intersections:      ', all)
    print('Only found real intersections:', real)
    print(findAllIntersects(lines))

    print('*'*28, 'Test createGraph', '*'*28)
    print('\nSee plot')
    G = graphFromLines(lines)

    print('*'*28, 'Test motion planning', '*'*28)
    sNode = Node(15, -2, str(lines[8][2:4]))
    eNode = Node(2, 1, str(lines[3][0:2]))
    breadthPath, breadthNodes = breadthFirstPathS(G, sNode, eNode)
    aPath, aNodes = aStarPathS(G, sNode, eNode)
    print('Breadth path: ', breadthPath)
    print('A* path: ', aPath)


    print('\nBreadth Distance: ', pathDistance(G, breadthPath))
    print('A* Distance: ', pathDistance(G, aPath))

    print('\nBreadth Directions: ', pathToDirections(breadthPath))
    print('A* Directions: ', pathToDirections(aPath))

    print('*'*28, 'Search Statistics', '*'*28)

    avBreadthDist, avAStarDist, avBreadthNodes, avAStarNodes, avBreadthTime, avAStarTime = statistics()

    print('Average distance for BFS vs aStar:', avBreadthDist, 'vs.', avAStarDist)
    print('Average number of nodes checked for BFS vs aStar:', avBreadthNodes, 'vs.', avAStarNodes)
    print('Average time taken for BFS vs aStar:', avBreadthTime, 'vs.', avAStarTime)

    drawGraph(G)
