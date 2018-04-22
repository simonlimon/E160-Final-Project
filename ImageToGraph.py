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
from skimage.transform import rescale
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

def show_point(img, x, y):
    fig, ax = plt.subplots()  
    ax.imshow(img, interpolation='nearest', cmap=plt.cm.gray)
    ax.plot([x], [y], 'sr', markersize=10)    
    plt.show()
    
def prepare_img(filename, resize = False):

    image = rescale(io.imread(filename, True), 1.0 / 5.0)
        
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
            if img[j][i] == True:
                return j, i

def cycle_match(G, x, y):
    rng = 10
    for n in G.nodes():
        between_x = n.x - rng < x and x < n.x + rng
        between_y = n.y - rng < y and y < n.y + rng
        if between_x and between_y:
            return n 
    return None

def traverse(G, img, box, parent_node):
    state = box.update()
    while(state == 'line'):
        state = box.update()
        print(box.x, box.y, state)    
    
    c_node = cycle_match(G, box.x, box.y)
    if c_node is not None:
        G.add_edge(parent_node, c_node, length=distance(c_node, parent_node))        
        return
    
    # show_point(img, box.x, box.y)                      
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
    SIZE = 10
    RSLV_STEP = 5
    
    def __init__(self, img, x, y, direction = None):
        self.img = img
        self.x = x
        self.y = y
        if direction is not None:
            self.direction = direction
        else:
            self.direction = self.determine_initial_dir()
            self.move(self.SIZE)
    
    def determine_initial_dir(self):
        lines, _ = self.detect_lines()
        for i in range(len(lines)):
            if lines[i] == 1:
                return i
    
    def intersection_copy(self, turn):
        new_dir = ({
            'front': 0,           
            'right': 1,
            'left': -1
        }.get(turn) + self.direction) % 4

        new_box = Box(self.img, self.x, self.y, new_dir)
        new_box.move(self.SIZE)
        return new_box
    
    def update(self, resolve = True):
        lines, line_pos = self.detect_lines()
        front = self.direction
        right = (self.direction + 1) % 4
        left = (self.direction - 1) % 4

        # print(lines)
        # show_point(self.img, self.x, self.y)                    

        if sum(lines) == 2: # follow line
            if lines[front] == 1:
                self.move()
                return 'line'
        elif resolve: # resolve intersection
            self.move(self.RSLV_STEP)      
            return self.update(False)

        if sum(lines) == 1:
            return 'dead_end'
        elif sum(lines) == 2:
            if lines[right] == 1:
                self.move_to_interception(line_pos, front, right)             
                return 'right_turn'
            elif lines[left] == 1:
                self.move_to_interception(line_pos, front, left)
                return 'left_turn'
        elif sum(lines) == 3:
            if lines[front] == 0:   
                self.move_to_interception(line_pos, front, left)                                        
                return 'threeway_left_right'
            elif lines[right] == 1:    
                self.move_to_interception(line_pos, front, right)                                       
                return 'threeway_right'
            elif lines[left] == 1:      
                self.move_to_interception(line_pos, front, left)
                return 'threeway_left'
        elif sum(lines) == 4:
            self.move_to_interception(line_pos, front, left)
            return 'fourways'

    def move_to_interception(self, line_pos, front, turn):
        back = (self.direction + 2) % 4
        if front % 2 == 0:
            self.x = line_pos[back]
            self.y = line_pos[turn]
        else:
            self.x = line_pos[turn]
            self.y = line_pos[back]

    def move(self, steps = 2):
        if self.direction == 0: self.y -= steps
        elif self.direction == 1: self.x += steps
        elif self.direction == 2: self.y += steps                                          
        elif self.direction == 3: self.x -= steps
    
    def detect_lines(self):
        lines = [0,0,0,0]
        line_pos = [0,0,0,0]        
        
        for x in range(max(0, self.x - self.SIZE), min(len(self.img[0])-1, self.x + self.SIZE)):
            y = min(self.y + self.SIZE, len(self.img)-1)
            if (self.img[y][x] == True):
                lines[2] = 1
                line_pos[2] = x
                break                

        for x in range(max(0, self.x - self.SIZE),  min(len(self.img[0])-1, self.x + self.SIZE)):
            y = max(self.y - self.SIZE, 0)
            if (self.img[y][x] == True):
                lines[0] = 1
                line_pos[0] = x                
                break                
        
        for y in range(max(0, self.y - self.SIZE),  min(len(self.img)-1, self.y + self.SIZE)): 
            x = min(self.x + self.SIZE, len(self.img[0])-1)

            # print(y)
            # print(self.img[y][self.x-self.SIZE:self.x+self.SIZE])
            if (self.img[y][x] == True):
                lines[1] = 1
                line_pos[1] = y                             
                break
        
        for y in range(max(0, self.y - self.SIZE),  min(len(self.img)-1, self.y + self.SIZE)):
            x = max(self.x - self.SIZE, 0)
            if (self.img[y][self.x - self.SIZE] == True):
                lines[3] = 1
                line_pos[3] = y                                      
                break               
        
        # print()
        return lines, line_pos

if __name__ == "__main__":
    main(sys.argv[1:])