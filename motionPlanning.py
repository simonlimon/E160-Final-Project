import networkx as nx
import math
import random
#import matplotlib.pyplot as plt
from Node import *

startNode = Node(0, 0, 0)
endNode = Node(0, 0, 0)
numNodes = 0
numExpanded = 0

# Create a random maze with the given number of nodes for testing.  It is
# guaranteed to be connected. And the maximum number of nodes that a node can
# be connected to is 4.  If hasLoops is true, then there will be a random number
# of extra edges added between 1 and min(numNodes/2, 10)
def randGraphMaze(numNodes, hasLoops):

    G = nx.Graph()

    # The grid boundaries are -maxX < x < maxX and -maxY < y < maxY
    maxX = 1000
    maxY = 1000

    newNode = Node(0,0,0)

    for i in range(numNodes):

        # Put the new node at a random place on the grid
        direction = ['vertical', 'horizontal']

        newX = 0
        newY = 0

        # Connect the node to the network appropriately
        if i > 0:
            while(True):
                candidate = random.choice(list(G.nodes))

                if len(list(G.neighbors(candidate))) < 4:

                    if random.choice(direction) == 'vertical':
                        newX = candidate.x
                        newY = (random.random() * maxY * 2) - maxY
                    else:
                        newX = (random.random() * maxY * 2) - maxY
                        newY = candidate.y

                    newNode = Node(newX, newY, i)

                    G.add_node(newNode)
                    G.add_edge(candidate, newNode, length=distance(candidate, newNode))

                    break
        else:
            newX = (random.random() * maxY * 2) - maxY
            newY = (random.random() * maxY * 2) - maxY

            newNode = Node(newX, newY, i)

            G.add_node(newNode)

        if i == 0:
            global startNode
            startNode = newNode
        elif i < 0.9*numNodes:
            global endNode
            endNode = newNode

    # Add the extra edges to create loops and therefore multiple paths to get to
    # the same point
    if hasLoops:
        numNewEdges = int(random.random() * numNodes/2)

        for i in range(numNewEdges):
            while(True):
                node1 = random.choice(list(G.nodes))
                node2 = random.choice(list(G.nodes))

                while(node1 in list(G.neighbors(node2))):
                    node2 = random.choice(list(G.nodes))

                if len(list(G.neighbors(node1))) < 4 and len(list(G.neighbors(node2))) < 4:
                    G.add_edge(node1, node2)
                    break
    return G, startNode, endNode

# Visualize a given graph
def drawMaze(G):

    options = {'node_color': 'red',
                'node_size': 100,
                'width': 3,
                'with_labels': True}

    plt.figure(6)
    nx.draw_kamada_kawai(G, **options)
    plt.title('kk')

    plt.show()

# Uses breadth first search starting at the start node to find the quickest
# path to the end node.  Returns a list with the nodes that must be visited in
# order from start to finish. Crashes if the endNode does not exist in G.
def breadthFirstPath(G, startNode, endNode):
    global numExpanded
    numExpanded = 0

    pathsDict = {startNode: [startNode]}
    isFound = False

    if endNode == startNode:
        isFound = True

    iter = nx.bfs_successors(G, startNode)
    current = next(iter)
    numExpanded += 1

    # iterate through the graph, remembering the path to get to each node
    while(not isFound):
        for node in current[1]:
            pathsDict[node] = pathsDict[current[0]] + [node]
            numExpanded += 1

        if endNode in current[1]:
            isFound = True
        else:
            current = next(iter)


    return pathsDict[endNode]

# Find the shortest path with A*
def aStarPath(G, startNode, endNode):
    global numExpanded
    numExpanded = 0

    return nx.astar_path(G, startNode, endNode, distance, 'length')

# Same as breadthFirstPath but returns the number of nodes expanded as well
def breadthFirstPathS(G, startNode, endNode):
    global numExpanded
    numExpanded = 0

    pathsDict = {startNode: [startNode]}
    isFound = False

    if endNode == startNode:
        isFound = True

    iter = nx.bfs_successors(G, startNode)
    current = next(iter)
    numExpanded += 1

    # iterate through the graph, remembering the path to get to each node
    while(not isFound):
        for node in current[1]:
            pathsDict[node] = pathsDict[current[0]] + [node]
            numExpanded += 1

        if endNode in current[1]:
            isFound = True
        else:
            current = next(iter)


    return pathsDict[endNode], numExpanded

# Same as breadthFirstPath but returns the number of nodes expanded as well
def aStarPathS(G, startNode, endNode):
    global numExpanded
    numExpanded = 0

    return nx.astar_path(G, startNode, endNode, distance, 'length'), numExpanded

# Calculate the distance between two nodes
def distance(nodeA, nodeB):
    global numExpanded
    numExpanded += 1

    return ((nodeA.x-nodeB.x)**2 + (nodeA.y-nodeB.y)**2)**0.5

# Calculate the total distance traveled for a path
def pathDistance(G, path):

    return sum(G[u][v].get('length', 1) for u, v in zip(path[:-1], path[1:]))

# Turn the given path into a list of directions
# 1 means turn right
# 0 means go straight
# -1 means go left
def pathToDirections(path):
    directions = []

    # Assumes that the initial direction is positive Y and that is the correct
    # direction to get to the next node
    for i in range(1,len(path)-1):
        # Compare the direction from this node to the next node with the
        # direction from the last node to this node to determine which way to
        # turn
        # Assumes that all lines are roughly vertical or horizontal
        dy = path[i+1].y - path[i].y
        dx = path[i+1].x - path[i].x

        isHorizontal = False
        isPositive = False

        # Check if it is horizontal
        if math.isclose(max(abs(dx), abs(dy)), abs(dx), abs_tol=0.00001):
            isHorizontal = True

            if dx > 0:
                isPositive = True

        elif dy > 0:
            isPositive = True

        # Check for the last traversal
        dyOld = path[i].y - path[i-1].y
        dxOld = path[i].x - path[i-1].x

        isOldHorizontal = False
        isOldPositive = False

        # Check if it is horizontal
        if math.isclose(max(abs(dxOld), abs(dyOld)), abs(dxOld), abs_tol=0.00001):
            isOldHorizontal = True

            if dxOld > 0:
                isOldPositive = True

        elif dyOld > 0:
            isOldPositive = True

        # Determine the direction to turn
        if isHorizontal ^ isOldHorizontal:
            # Was going in the +X or -Y direction
            if not (isOldHorizontal ^ isOldPositive):
                # Will go in the +Y or +X direction
                if isPositive:
                    directions += [-1]
                # Will go in the -Y or -X direction
                else:
                    directions += [1]

            # Was going in the -X or +Y direction
            elif isOldHorizontal ^ isOldPositive:
                # Will go in the +Y or +X direction
                if isPositive:
                    directions += [1]
                # Will go in the -Y or -X direction
                else:
                    directions += [-1]

        # If both edges are horizontal or both are vertical then go straight
        # through the node
        else:
            directions += [0]

    return directions



def main():
    global numNodes
    numNodes = 50

    G = randGraphMaze(numNodes, True)
    breadthPath = breadthFirstPath(G, startNode, endNode)
    aPath = aStarPath(G, startNode, endNode)
    print('Breadth path: ', breadthPath)
    print('A* path: ', aPath)


    print('\nBreadth Distance: ', pathDistance(breadthPath))
    print('A* Distance: ', pathDistance(aPath))

    print('\nBreadth Directions: ', pathToDirections(breadthPath))
    print('A* Directions: ', pathToDirections(aPath))

    drawMaze(G)


if __name__ == "__main__":
    main()
