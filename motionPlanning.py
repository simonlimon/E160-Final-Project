import networkx as nx
import math
import random
import matplotlib.pyplot as plt
from Node import *

startNode = Node(0, 0, 0)
endNode = Node(0, 0, 0)
numNodes = 0

# Create a random maze with the given number of nodes for testing.  It is
# guaranteed to be connected. And the maximum number of nodes that a node can
# be connected to is 4.  If hasLoops is true, then there will be a random number
# of extra edges added between 1 and min(numNodes/2, 10)
def randGraphMaze(numNodes, hasLoops):

    G = nx.Graph()

    # The grid boundaries are -maxX < x < maxX and -maxY < y < maxY
    maxX = 1000
    maxY = 1000

    for i in range(numNodes):

        # Put the new node at a random place on the grid
        newX = (random.random() * maxX * 2) - maxX
        newY = (random.random() * maxY * 2) - maxY
        newNode = Node(newX, newY, i)

        # Connect the node to the network appropriately
        if i > 0:
            while(True):
                candidate = random.choice(list(G.nodes))

                if len(list(G.neighbors(candidate))) < 4:

                    G.add_node(newNode)
                    G.add_edge(candidate, newNode, length=distance(candidate, newNode))

                    break
        else:
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
    return G

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
def breadthFirstDirections(G, startNode, endNode):
    pathsDict = {startNode: [startNode]}
    isFound = False

    if endNode == startNode:
        isFound = True

    iter = nx.bfs_successors(G, startNode)
    current = next(iter)

    # iterate through the graph, remembering the path to get to each node
    while(not isFound):
        for node in current[1]:
            pathsDict[node] = pathsDict[current[0]] + [node]

        if endNode in current[1]:
            isFound = True
        else:
            current = next(iter)

    return pathsDict[endNode]

# Find the shortest path with A*
def aStarDirections(G, startNode, endNode):

    return nx.astar_path(G, startNode, endNode, heuristic=distance, weight='length')

# Calculate the distance between two nodes
def distance(nodeA, nodeB):
    return ((nodeA.x-nodeB.x)**2 + (nodeA.y-nodeB.y)**2)**0.5

# Calculate the total distance traveled for a path
def pathDistance(path):
    dist = 0

    for i in range(1, len(path)):
        dist += distance(path[i-1], path[i])

    return dist

def main():
    global numNodes
    numNodes = 50

    G = randGraphMaze(numNodes, True)
    breadthPath = breadthFirstDirections(G, startNode, endNode)
    aStarPath = aStarDirections(G, startNode, endNode)
    print('Breadth path: ', breadthPath)
    print('A* path: ', aStarPath)



    print('\n Breadth Distance: ', pathDistance(breadthPath))
    print('A* Distance: ', pathDistance(aStarPath))

    drawMaze(G)


if __name__ == "__main__":
    main()
