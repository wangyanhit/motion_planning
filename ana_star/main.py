import sys
from PIL import Image
import copy
from heapq import heappush, heappop
import numpy as np
import matplotlib.pyplot as plt
import time

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = "" # a string reference to the original import file
im_size = None
algorithm = "ANA*"
free_bit = 0 # for map provided by TA
'''
These variables determine display coler, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)
'''
NEON_GREEN = (0, 0, 255)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (250, 0, 0)
DARK_GRAY = (0, 250, 0)
'''

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}   # a dictionary of (x,y) tuples, representing nodes to expand to in the future


def distance(a, b):
    x = a[0] - b[0]
    y = a[1] - b[1]
    # Euclidian distance
    return np.sqrt(x**2 + y**2) + 0.001


def action_cost(a, b):
    return 1



def search(map):
    """
    This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
    provided map.
    :param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    """

    # O is unoccupied (white); 1 is occupied (black)
    print "pixel value at start point ", map[start[0], start[1]]
    print "pixel value at end point ", map[end[0], end[1]]

    # put your final path into this array (so visualize_search can draw it in purple)
    #path.extend([(8,2), (8,3), (8,4), (8,5), (8,6), (8,7)])

    # put your expanded nodes into this dictionary (so visualize_search can draw them in dark gray)
    #expanded.update({(7,2):True, (7,3):True, (7,4):True, (7,5):True, (7,6):True, (7,7):True})

    # put your frontier nodes into this dictionary (so visualize_search can draw them in light gray)
    #frontier.update({(6,2):True, (6,3):True, (6,4):True, (6,5):True, (6,6):True, (6,7):True})
    x_max, y_max = im_size
    print "map size: ", im_size
    action = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    false_infinite = 1.0e10
    pred = {}
    g = np.ones(im_size, dtype=np.float) * false_infinite
    g[start] = 0
    found_num = 0
    push_cnt = 0
    pop_cnt = 0


    start_time = time.time()

    if algorithm == "A*":
        open = [(distance(start, end), start)]

        while open:
            s = heappop(open)
            pop_cnt += 1
            pos = s[1]
            if pos == end:
                p = pred[end]
                while p != start:
                    path.append(p)
                    p = pred[p]
                found_num += 1
                print "find path"
                break

            for a in action:
                s_prime_pos = (pos[0] + a[0], pos[1] + a[1])
                # check if the node is out of boundary
                if s_prime_pos[0] >= 0 and s_prime_pos[0] < x_max and s_prime_pos[1] >= 0 and s_prime_pos[1] < y_max:
                    # check if the node is obstacle
                    if map[s_prime_pos] == free_bit:
                        expanded[s_prime_pos] = True
                        if g[pos] + action_cost(pos, s_prime_pos) < g[s_prime_pos]:
                            g[s_prime_pos] = g[pos] + action_cost(pos, s_prime_pos)
                            pred[s_prime_pos] = pos
                            in_open = False
                            for i in range(len(open)):
                                _, pos_o = open[i]
                                if pos_o == s_prime_pos:
                                    in_open = True
                                    open[i] = (g[s_prime_pos] + distance(s_prime_pos, end), s_prime_pos)
                                    break
                            if not in_open:
                                heappush(open, (g[s_prime_pos] + distance(s_prime_pos, end), s_prime_pos))
                                push_cnt += 1
        for o in open:
            frontier[o[1]] = True

    else:
        G = false_infinite
        E = false_infinite
        open = [(-(G - g[start])/distance(start, end), start)]
        while open:
            # improve solution
            while open:
                s = heappop(open)
                pop_cnt += 1
                e_s, pos = s
                e_s = -e_s
                if e_s < E:
                    E = e_s
                if pos == end:
                    G = g[pos]
                    p = pred[end]
                    while p != start:
                        path.append(p)
                        p = pred[p]
                    found_num += 1
                    print "find {} path".format(found_num)
                    break
                for a in action:
                    s_prime_pos = (pos[0]+a[0], pos[1]+a[1])
                    # check if the node is out of boundary
                    if s_prime_pos[0] >= 0 and s_prime_pos[0] < x_max and s_prime_pos[1] >= 0 and s_prime_pos[1] < y_max:
                        #print s_prime_pos
                        # check if the node is obstacle
                        if map[s_prime_pos] == free_bit:
                            expanded[s_prime_pos] = True
                            if g[pos] + action_cost(pos, s_prime_pos) < g[s_prime_pos]:
                                g[s_prime_pos] = g[pos] + action_cost(pos, s_prime_pos)
                                pred[s_prime_pos] = pos
                                if g[s_prime_pos] + distance(s_prime_pos, end) < G:
                                    in_open = False
                                    for i in range(len(open)):
                                        _, pos_o = open[i]
                                        if pos_o == s_prime_pos:
                                            in_open = True
                                            open[i] = (-(G - g[s_prime_pos])/distance(s_prime_pos, end), s_prime_pos)
                                            break
                                    if not in_open:
                                        heappush(open, (-(G - g[s_prime_pos])/distance(s_prime_pos, end), s_prime_pos))
                                        push_cnt += 1
            if found_num > 0:
                break
            print "open len", len(open)
            open2 = []
            for o in open:
                e_o, pos_o = o
                e_o = -e_o
                if g[pos_o] + distance(pos_o, end) >= G:
                    continue
                heappush(open2, (-(G - g[pos_o])/distance(pos_o, end), pos_o))
            open = copy.copy(open2)


        for o in open:
            frontier[o[1]] = True
    if found_num == 0:
        print "No path found!"
    print "push count: {}, pop count: {}".format(push_cnt, pop_cnt)
    print "path length: {}".format(g[end])
    print "cost time: {} s".format(time.time() - start_time)
    '''
    YOUR WORK HERE.
    
    I believe in you
        -Gunnar (the TA)-
    '''
    #print "path", path
    #print "expanded", expanded
    #print "frontier", frontier
    visualize_search("out.png") # see what your search has wrought (and maybe save your results)

def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN


    # draw frontier pixels
    for pixel in frontier.keys():
        pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # display and (maybe) save results
    im.show()
    if(save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()


if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)
    assert len(sys.argv) == 2 or len(sys.argv) == 3, "Incorrect Number of arguments"      # require difficulty input

    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    algorithm = str(sys.argv[2])
    print algorithm
    if algorithm != "A*":
        algorithm = "ANA*"
    print "running " + function_name + " with " + difficulty + " difficulty using " + algorithm +" algorithm."

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "b_map1.gif":
        free_bit = 255
        start = (14, 14)
        end = (16, 57)
    elif difficulty == "b_map2.png":
        free_bit = 255
        start = (37, 28)
        end = (540, 480)
    else:
        assert False, "Incorrect difficulty level provided"

    # Perform search on given image
    im = Image.open(difficulty)
    im_size = im.size
    search(im.load())
