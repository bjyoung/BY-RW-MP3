import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import copy
import Queue

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()
    
    # Your code goes here
    
    return newPoints, adjListMap

'''
Perform basic search 
'''
class Node:
    """
    Represents a node in the graph

    Attr:
        label: label of this node
        parent: previously visited Node before reaching current one, None by default
    """

    def __init__(self, label):
        self.label = label
        self.parent = None

    def __eq__(self, other):
        """
        Two cells are equivalent if their labels are equivalent
        """
        if not isinstance(other, Node):
            return False

        if self.label == other.label:
            return True
        return False

    def __str__(self):
        """
        Prints out Node in the format (label, parent's label, f)
        """
        parent_str = 'None'
        if self.parent is not None:
            parent = self.parent.label

        return "({0}, parent={1})".format(self.label, parent_str)

    def __hash__(self):
        """
        Hash this node

        :return: for node i, hash its label, i
        """
        return hash(self.label)

def contained_in_closed(label, closed):
    """
    Check if given point is in closed dictionary or not

    :param label: label of the node
    :param closed: closed dictionary
    :return: True if point is contained in the dictionary, False otherwise
    """
    if label in closed:
        if label in closed[label]:
            return True
    return False

def retrieve_path(start, goal, nodes_dict):
    """
    Find the path leading from start to goal by working backwards from the goal

    Parameters:
    start: label for the start vertex
    goal: label for goal vertex
    nodes_dict: dictionary with labels as keys and the corresponding vertex's node as values

    Returns:
    1D array of labels to follow from start to goal
    """
    curr_node = nodes_dict[goal]
    path = [curr_node.label]  # Start at goal

    while curr_node.label != start:
        parent = curr_node.parent
        path.append(parent.label)
        curr_node = parent

    path.reverse()  # Reverse path so it starts at start and ends at goal
    return path

def basicSearch(tree, start, goal):
    path = []
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.

    if len(tree) == 0: # No tree, then no path
        path = None
        return path

    # Create dictionary of {labels:nodes}
    labels = tree.keys()[:]
    nodes = [Node(label) for label in labels]  # Create a node for every key

    nodes_dict = {} # Mapping from point label to its nodes
    for node in nodes:
        nodes_dict[node.label] = node

    # Run BFS
    start_node = nodes_dict[start]
    start_node.parent = start
    queue = Queue.Queue()
    queue.put(start)  # Insert start to queue
    closed = {}  # closed := empty dictionary

    while not queue.empty():  # Checking that queue is nonempty
        s = queue.get()
        s_node = nodes_dict[s]

        if s_node.label == goal:
            path = retrieve_path(start, goal, nodes_dict)  # Get path from start to goal
            return path

        # Store in closed dictionary
        if s_node.label in closed:
            closed[s_node.label].append(s_node.label)
        else:
            closed[s_node.label] = [s_node.label]

        # Get neighbors, if neighbor not in closed, then make s its parent and add to queue
        neighbors = tree[s_node.label]

        for i in range(len(neighbors)):
            neighbor = neighbors[i]
            neighbor_node = nodes_dict[neighbor]

            if not contained_in_closed(neighbor, closed):
                neighbor_node.parent = s_node
                queue.put(neighbor)

    path = None
    return path

'''
Display the RRT and Path
'''
def plotTree(points, tree):
    """
    Plot roadmap in black

    :param points: mapping from point id to its coordinates
    :param tree: RRT as mapping from point id to its neighbors' ids
    :return: None
    """
    if tree is None:
        return

    tree_color = 'black'

    for id in tree.keys():
        point = points[id]

        # Find all neighbors labels
        neighbors = adjListMap[id]

        # Draw line segment from inital point to its neighors
        for neighbor_id in neighbors:
            neighbor_pt = points[neighbor_id]
            tree_x = [point[0], neighbor_pt[0]]
            tree_y = [point[1], neighbor_pt[1]]
            plt.plot(tree_x, tree_y, tree_color)

def plotPath(points, path):
    """
    Plot path in orange. If there is no path, plot nothing
    :param points: mapping from point id to coordinates
    :param path: list of ids leading from start to goal
    :return: None
    """
    if path is None:
        return

    path_color = 'orange'

    # Add path to plot
    path_x = []
    path_y = []

    for label in path:
        pt = points[label]
        path_x.append(pt[0])
        path_y.append(pt[1])
    plt.plot(path_x, path_y, path_color)

def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):
    """
    Plot the given robot start & goal, obstacles, tree and path.
    Tree has black edges.
    Path has orange edges.

    Parameters:
    points = mapping from point ids to their xy-coordinates as 2-tuples
    tree = RRT as mapping from point id to its neighbors' ids
    path = list of point ids from start to goal
    robotStart = xy-tuple of start
    robotGoal = xy-tuple of goal
    polygons = list of obstacles, where an obstacle is represented as a list of cw coordinates of the vertices of an obstacle
    Returns: None
    """
    # From drawProblem() (modified)
    fig, ax = setupPlot()

    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)

    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)

    if polygons is not None:
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)
    # From drawProblem() (modified)

    # Normalize all vertex coordinates (because polygons are outputted in 1x1 box, not 10x10 box)
    points_copy = copy.deepcopy(points)
    for label in points_copy.keys():
        point = points_copy[label]
        points_copy[label] = [point[0]/10.0, point[1]/10.0]

    # Plot data and show
    plotTree(points_copy, tree)
    plotPath(points_copy, path)

    title = "RRT & Path"
    plt.title(title)
    plt.grid(True)
    plt.show()

    return 

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
    
    return False

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []
    # Your code goes here.
    
    return points, tree, path

if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    """ FOR TESTING REMOVE LATER--------------------------------------------------------------------------
    filename = "robot_env_01.txt"
    x1 = 1.0
    y1 = 2.0
    x2 = 8.5
    y2 = 7

    adjListMap = {0: [1, 2], 1: [2, 3, 4], 2: [3], 3: [4], 4: []}
    path = basicSearch(adjListMap, 4, 3)
    print "path = {}".format(path)
    """

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print "" 
    print "The input points are:"
    print str(points)
    print ""
    
    points, adjListMap = growSimpleRRT(points)

    # Search for a solution  
    path = basicSearch(adjListMap, 1, 20)    

    # Your visualization code 
    displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)



