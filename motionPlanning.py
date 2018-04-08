import networkx as nx
import math
import random
import matplotlib.pyplot as plt

# Create a random maze with the given number of nodes for testing.  It is
# guaranteed to be connected. And the maximum number of nodes that a node can
# be connected to is 4.  If hasLoops is true, then there will be a random number
# of extra edges added between 1 and min(numNodes/2, 10)
def randGraphMaze(numNodes, hasLoops):

    G = nx.Graph()

    for i in range(numNodes):

        # Connect the node to the network appropriately
        if i > 0:
            while(True):
                candidate = random.choice(list(G.nodes))

                if len(list(G.neighbors(candidate))) < 4:
                    G.add_node(i)
                    G.add_edge(candidate, i)
                    break
        else:
            G.add_node(i)

    # Add the extra edges to create loops
    if hasLoops:
        numNewEdges = int(random.random() * min(numNodes/2, 10))

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


def main():
    G = randGraphMaze(50, True)
    print('The path: ', breadthFirstDirections(G, 1, 29))

    drawMaze(G)


if __name__ == "__main__":
    main()
