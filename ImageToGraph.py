"""
https://docs.opencv.org/3.4.0/d7/d4d/tutorial_py_thresholding.html
"""
import sys
import math
import numpy as np
from matplotlib import pyplot as plt
from Node import Node
import networkx as nx
from skimage.filters import threshold_otsu, gaussian
from skimage.morphology import skeletonize
from skimage.util import invert
from skimage.transform import resize
from skimage import io


def main(argv):

    img = prepare_img(argv[0])
    G = build_graph(img)

    pos = {}
    for n in G.nodes():
        pos[n] = (n.x, n.y)
    
    fig, ax = plt.subplots()  
    ax.imshow(img, interpolation='nearest', cmap=plt.cm.gray)
    nx.draw(G,pos=pos, node_shape='+', edge_color='b')        
    plt.show()
    
def prepare_img(filename):
    image = resize(io.imread(filename, True), (300, 300))
    binary = image > threshold_otsu(image)
    skeleton = skeletonize(invert(binary))
    return skeleton

def build_graph(img): 
    G = nx.Graph()
    y, x = find_start(img)

    # create starting box
    node = Node(x, y, str((x,y)))        
    G.add_node(node)

    box = Box(img, x, y)    
    traverse(G, img, box, node)

    return G

def find_start(img):
    for i in range(len(img)):
        for j in range(len(img)):
            if img[i, j] == True:
                return i, j

def traverse(G, img, box, parent_node):
    state = box.update()
    while(state == 'line'):
        state = box.update()
    print(box.x, box.y, state)
    
    node = append_to_grap(G, parent_node, box.x, box.y)

    if state == 'dead_end':
        return 
    elif state == 'right_turn':
        traverse(G, img, box.intersection_copy('right') , node)  
    elif state == 'left_turn':
        traverse(G, img, box.intersection_copy('left') , node)  
    elif state == 'threeway_right':
        traverse(G, img, box.intersection_copy('front'), node)
        traverse(G, img, box.intersection_copy('right') , node)  
    elif state == 'threeway_left':
        traverse(G, img, box.intersection_copy('front'), node)
        traverse(G, img, box.intersection_copy('left'), node)  
    elif state == 'threeway_left_right':
        traverse(G, img, box.intersection_copy('right'), node)
        traverse(G, img, box.intersection_copy('left'), node)        
    elif state == 'fourways':  
        traverse(G, img, box.intersection_copy('front'), node)
        traverse(G, img, box.intersection_copy('right'), node)   
        traverse(G, img, box.intersection_copy('left') , node)                                           
           
def append_to_grap(G, parent, x, y):
    node = Node(x, y, str((x,y)))        
    G.add_node(node)
    G.add_edge(parent, node, length=distance(node, parent))
    return node

def distance(nodeA, nodeB):
    return ((nodeA.x-nodeB.x)**2 + (nodeA.y-nodeB.y)**2)**0.5

class Box():
    SIZE = 3
    
    def __init__(self, img, x, y, direction = None):
        self.img = img
        self.x = x
        self.y = y
        if direction is not None:
            self.direction = direction
        else:
            self.direction = self.determine_initial_dir()
            self.move(self.SIZE*2)
    
    def determine_initial_dir(self):
        lines = self.detect_lines()
        for i in range(len(lines)):
            if lines[i] == 1:
                return i
    
    def intersection_copy(self, turn):
        new_dir = ({
            'front': 0,           
            'right': 1,
            'left': -1
        }.get(turn) + self.direction) % 4

        new_box = Box(self.img, self.x, self.y, self.direction)
        new_box.move(self.SIZE-1)
        new_box.direction = new_dir
        new_box.move(self.SIZE+1)
        return new_box
        
    
    def update(self, resolve = True):
        lines = self.detect_lines()
        front = lines[self.direction]
        right = lines[(self.direction + 1) % 4]
        left = lines[(self.direction - 1) % 4]
        if sum(lines) == 2:
            if front == 1:
                self.move()
                return 'line'
        elif resolve:
            self.move()                
            return self.update(False)
            
        if sum(lines) == 1:
            return 'dead_end'
        elif sum(lines) == 2:
            if right == 1:
                return 'right_turn'
            elif left == 1:
                return 'left_turn'
        elif sum(lines) == 3:
            if front == 0:
                return 'threeway_left_right'
            elif right == 1:
                return 'threeway_right'
            elif left == 1:
                return 'threeway_left'
        elif sum(lines) == 4:
            return 'fourways'
        
        
    def move(self, steps = 1):
        if self.direction == 0: self.y -= steps
        elif self.direction == 1: self.x += steps
        elif self.direction == 2: self.y += steps                                          
        elif self.direction == 3: self.x -= steps
    
    def detect_lines(self):
        lines = [0,0,0,0]
        
        for x in range(max(0, self.x - self.SIZE), min(len(self.img[0])-1, self.x + self.SIZE)):
            y = min(self.y + self.SIZE, len(self.img)-1)
            if (self.img[y][x] == True):
                lines[2] = 1
                break                

        for x in range(max(0, self.x - self.SIZE),  min(len(self.img[0])-1, self.x + self.SIZE)):
            y = max(self.y - self.SIZE, 0)
            if (self.img[y][x] == True):
                lines[0] = 1
                break                
        
        for y in range(max(0, self.y - self.SIZE),  min(len(self.img)-1, self.y + self.SIZE)): 
            x = min(self.x + self.SIZE, len(self.img[0])-1)

            # print(y)
            # print(self.img[y][self.x-self.SIZE:self.x+self.SIZE])
            if (self.img[y][x] == True):
                lines[1] = 1
                break
        
        for y in range(max(0, self.y - self.SIZE),  min(len(self.img)-1, self.y + self.SIZE)):
            x = max(self.x - self.SIZE, 0)
            if (self.img[y][self.x - self.SIZE] == True):
                lines[3] = 1
                break                
        
        # print()
        return lines

if __name__ == "__main__":
    main(sys.argv[1:])