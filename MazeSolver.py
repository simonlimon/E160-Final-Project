import sys
import time

from E160_gui import gui_main
from ImageToGraph import build_graph, prepare_img, select_start_end
from motionPlanning import aStarPath, breadthFirstPath, pathDistance, pathToDirections

def main(filename):
    start_time = time.time()
    img = prepare_img(filename, True)
    print('Time to prepare image: ', time.time() - start_time)

    start_time = time.time()
    G = build_graph(img)
    print('Time to build graph: ', time.time() - start_time)

    start, end = select_start_end(G, img)

    breadthPath = breadthFirstPath(G, start, end)
    aPath = aStarPath(G, start, end)
    print('BFS path: ', breadthPath)
    print('A* path: ', aPath)


    print('\BFS Distance: ', pathDistance(G, breadthPath))
    print('A* Distance: ', pathDistance(G, aPath))

    print('\BFS Directions: ', pathToDirections(breadthPath))
    print('A* Directions: ', pathToDirections(aPath))

    gui_main(pathToDirections(aPath))

if __name__ == '__main__':
    main(sys.argv[1])
