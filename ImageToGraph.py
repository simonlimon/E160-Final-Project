"""
https://docs.opencv.org/3.4.0/d7/d4d/tutorial_py_thresholding.html
"""
import sys
import math
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from Node import Node
import networkx as nx

def main(argv):

    filename = argv[0]

    # Loads an image
    src = cv.imread(filename, cv.IMREAD_GRAYSCALE)
    resized = cv.resize(src, (300, 300))

    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        return -1
    
    ret,thresh = cv.threshold(resized,127,255,cv.THRESH_BINARY)

    build_graph(thresh)

    # cv.imshow('binary', thresh)
    # cv.waitKey()    

def build_graph(thresh):
    G = nx.Graph()

    for y in range(len(thresh)-1):
        for x in range(len(thresh[y])-1):
            node = Node(x, y, str((x,y)))  
            if thresh[y][x] != 0: continue   

            G.add_node(node)
            if thresh[y+1][x] == 0:
                neighbor = Node(x, y, str((x,y)))          
                G.add_edge(node, neighbor, length=distance(node, neighbor))
            if thresh[y][x+1] == 0:
                neighbor = Node(x, y, str((x,y)))          
                G.add_edge(node, neighbor, length=distance(node, neighbor))

    # node = None
    # for node in G:
    #     if G.degree(node) > 2:
    #         break

    # queue = [node]
    # while (len(queue) > 0):
    #     node = queue.pop()
    #     for neighbor in G.neighbors(node):
    #         if G.degree(neighbor) <= 2:
    #             queue.append(node)             
    #             G = nx.contracted_nodes(G, node, neighbor, False)
    #         else:
    #             queue.append(neighbor)
            
    # nx.draw(G)
    # plt.show()

def distance(nodeA, nodeB):
    return ((nodeA.x-nodeB.x)**2 + (nodeA.y-nodeB.y)**2)**0.5
    
if __name__ == "__main__":
    main(sys.argv[1:])