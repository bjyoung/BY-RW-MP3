import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import copy
import Queue
import math
import random
import timeit

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
def get_euclidean_distance(point1, point2):
    """
    Find the euclidean distance between the given points

    :param point1: 2-tuple of point 1's coordinates
    :param point2: 2-tuple of point 2's coordinates

    :return: euclidean distance between the 2 points
    """
    x1 = point1[0]
    y1 = point1[1]
    x2 = point2[0]
    y2 = point2[1]
    dist = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
    return dist

def find_closest_point(new_point, points, adjListMap):
    """
    Find the closest vertex to the given point based on the adjListMap

    :param new_point: label of new point to add to map
    :param points: map point labels to their coordinates
    :param adjListMap: map from point labels to their neighbors

    :return: label of the closest point and the distance to reach that point
    """
    closest_point = -1
    min_dist = 5000 # represents infinity

    for label in adjListMap:
        vertex = points[label]
        dist = get_euclidean_distance(new_point, vertex)

        if dist < min_dist:
            closest_point = label
            min_dist = dist

    return closest_point, min_dist

def has_no_edges(adjListMap):
    """
    Detects if the adjListMap has edges or not

    :param adjListMap: map from point label to its neighbors
    :return: True if there no edges in the map, False otherwise
    """
    for label in adjListMap:
        if len(adjListMap[label]) != 0: # Has an edge
            return False

    # No edges
    return True

def calculate_slope_intercept(pt, slope):
    """
    Find the y-intercept of the line with the given slope and that goes through the given point

    :param pt: point that line goes through
    :param slope: slope of the line
    :return: y-intercept of the line
    """
    x = pt[0]
    y = pt[1]
    b = y - slope * x
    return b

def get_line_equation(a, b):
    """
    Find the slope and y-intercept of the line formed by the given points

    :param a: point a (2-tuple)
    :param b: point b

    :return: slope and y-intercept of the line
    """
    x1 = a[0]
    y1 = a[1]
    x2 = b[0]
    y2 = b[1]

    m = ((y2 - y1) * 1.0)/(x2 - x1)
    b = calculate_slope_intercept(a, m)
    return m, b

def get_distance_to_edge(new_point, edge, points):
    """
    Detects if the adjListMap has edges or not

    :param new_point: label of new point to add to map
    :param edge: 2-tuple of point labels of the edge's endpoints
    :param points: map from point label to the point's coordinates
    :return: distance to the edge and the intersecting point (which was used to find the distance)

    """
    new_point_coord = points[new_point]
    edge_endpt_1 = points[edge[0]]
    edge_endpt_2 = points[edge[1]]

    # Handle vertical line case separately
    if edge_endpt_1[0] == edge_endpt_2[0]:
        x = edge_endpt_1[0]
        intersect_pt = (x, new_point_coord[1]) # Take x-value of either point and y-value of new point
        dist = math.fabs(x - new_point_coord[0]) # Take absolute difference between x-values
        return dist, intersect_pt
    # Handle horizontal line separately
    elif edge_endpt_1[1] == edge_endpt_2[1]:
        y = edge_endpt_1[1]
        intersect_pt = (new_point_coord[0], y)
        dist = math.fabs(y - new_point_coord[1])
        return dist, intersect_pt

    # Find equation of the line for the edge
    m1, b1 = get_line_equation(edge_endpt_1, edge_endpt_2)

    # Find equation of perpendicular line that goes through the new point
    m2 = -1.0/m1
    b2 = calculate_slope_intercept(new_point_coord, m2)

    # Find where the edge and the perpendicular line intersect
    x_intersect = ((b2 - b1) * 1.0)/(m1 - m2)
    y_intersect = m1 * x_intersect + b1
    intersect_pt = (x_intersect, y_intersect)
    dist = get_euclidean_distance(new_point_coord, intersect_pt)
    return dist, intersect_pt

def is_between_endpts(intersect_pt, edge):
    """
    Determine whether the intersection point is located between the endpoints along the edge or not
    :param intersect_pt: 2-tuple of coordinates for the intersection point
    :param edge: 2-list of the edge's endpoint coordinates

    :return: True if intersect_pt is located between the endpoints of the edge, False otherwise
    """
    endpt_a = edge[0]
    endpt_b = edge[1]
    c = intersect_pt

    dist_ac = get_euclidean_distance(endpt_a, c)
    dist_cb = get_euclidean_distance(c, endpt_b)
    dist_ab = get_euclidean_distance(endpt_a, endpt_b)

    epsilon = 0.01 # Because equivalence may not work with decimals
    return True if math.fabs(dist_ab - (dist_ac + dist_cb)) <= 0.01 else False

def find_closest_edge(new_point, closest_point, points, adjListMap):
    """
    Find the closest edge to the given point based on the adjListMap
    If there are no edges on the map, return -1 as the point and 5000 as the min_dist
    If there is an edge, give the two endpoints that form the edge and the intersecting pt

    :param new_point: label of new point to add to map
    :param closest_point: label of the closest point in the tree to the new point
    :param points: map point labels to their coordinates
    :param adjListMap: map from point labels to their neighbors

    :return: closest edge (as 2-tuple of labels of its endpoints) and the distance from new point to the edge
    """
    closest_edge = [-1, -1]
    min_dist = 5000 # represents infinity, since 10x10 grid
    closest_intersect_pt = (-1, -1)

    neighbor_labels = adjListMap[closest_point]
    if not neighbor_labels: # Check if there are any edges from closest_point
        return closest_edge, min_dist, closest_intersect_pt

    # For every edge (label-neighbor pair) from the closest point, find dist from new point to the edge
    # Update min edge if the distance found is smaller
    for neighbor_label in neighbor_labels:
        edge = [closest_point, neighbor_label]
        dist, intersect_pt = get_distance_to_edge(new_point, edge, points)

        # Check if intersect pt is between edge's endpoints, if not, then throw out edge
        edge_coordinates = [points[closest_point], points[neighbor_label]]
        if not is_between_endpts(intersect_pt, edge_coordinates):
            continue

        if dist < min_dist:
            closest_edge = edge
            min_dist = dist
            closest_intersect_pt = intersect_pt

    return closest_edge, min_dist, closest_intersect_pt

def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()

    # Your code goes here
    if points is None:
        return newPoints, adjListMap

    newPoints = copy.deepcopy(points)
    numPoints = len(newPoints)

    for label in points:
        point = points[label]

        if not adjListMap: # Checks if adjListMap is empty
            adjListMap[label] = [] # Add point as starting node (has no neighbors to start off)
            continue

        # Find closest point and edge
        closest_point, closest_pt_dist = find_closest_point(point, newPoints, adjListMap)
        closest_edge, closest_edge_dist, closest_intersect_pt = find_closest_edge(label, closest_point, newPoints, adjListMap)

        # If point, then add as neighbor to the closest point
        if closest_pt_dist <= closest_edge_dist:
            adjListMap[closest_point].append(label)
            adjListMap[label] = [closest_point]

        # If edge, then create a new point and update adjListMap and newPoints
        else:
            # Add intersection point coordinates to points
            intersect_label = numPoints + 1
            newPoints[intersect_label] = closest_intersect_pt
            numPoints += 1

            # Update endpoint neighbors and add adjList for intersection and new points
            endpt_a = closest_edge[0]
            endpt_b = closest_edge[1]

            adjListMap[endpt_a].remove(endpt_b)
            adjListMap[endpt_a].append(intersect_label)

            adjListMap[endpt_b].remove(endpt_a)
            adjListMap[endpt_b].append(intersect_label)

            adjListMap[label] = [intersect_label]

            adjListMap[intersect_label] = [endpt_a, endpt_b, label]

        # # FOR TESTING, visualize tree after every step
        #displayRRTandPath(newPoints, adjListMap, None)

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
        neighbors = tree[id]
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
def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def isCollisionFree(robot, point, obstacles):
    # Your code goes here.
    # xCor/yCor : (x1,y1)
    xCor = point[0]
    yCor = point[1]


    #global robot points : [(x1,y1),(x2,y2),(x3,y3)...]
    globalRobot =[]
    for pt in robot:
        tempPoint = (pt[0]+xCor,pt[1]+yCor)
        globalRobot.append(tempPoint)

    globalEdge = []
    i = 0
    while i < len(globalRobot):
        if i == len(globalRobot) - 1:
            globalEdge.append((globalRobot[i], globalRobot[0]))
        else:
            globalEdge.append((globalRobot[i], globalRobot[i + 1]))
        i += 1
    #collision or not, since it is asking isCollisionFree
    #if collision then we should return false,means not collision free
    #else true
    edgeGroup=[]
    collision = False
    for obs in obstacles:
        i=0
        while i < len(obs):
            if i == len(obs)-1:
                edgeGroup.append((obs[i],obs[0]))
            else:
                edgeGroup.append((obs[i],obs[i+1]))
            i+=1
        obsPath = Path(np.array(obs))
        for pt in globalRobot:
            collision = collision or obsPath.contains_point(pt)
            #as long as any point is in the polygon,we have collision
    #add boundary
    edgeGroup.append(((0,0),(0,10)))
    edgeGroup.append(((0,0),(10,0)))
    edgeGroup.append(((10,0),(10,10)))
    edgeGroup.append(((0,10),(10,10)))

    # need also check if every edge intersects with polygons
    for edge in globalEdge:
        for obsEdge in edgeGroup:
            collision = collision or intersect(obsEdge[0], obsEdge[1], edge[0], edge[1])

    return not collision

def randSamplePoint(xBound,yBound):
    return (random.uniform(0, xBound),random.uniform(0, yBound))

def edgePointInterval(start,end):
    if(start[0]!=end[0]):
        m,b=get_line_equation(start,end)
    else:
        m,b=0,0
    INTERVEL = 20.0
    if m==b and b==0:
        INCREMENT = (end[1]-start[1])/20.0
    else:
        INCREMENT = (end[0]-start[0])/20.0
    points=[]
    for i in range(20):
        if m==b and b==0:
            xCor = start[0]
            yCor = start[1]+i*INCREMENT
        else:
            xCor = start[0]+INCREMENT*i
            yCor = m*xCor+b
        points.append((xCor,yCor))
    points.append(end)
    return points

def growComplexRRT(new_point, points, tree,OBSedgeGroup):
    """
    :param new_point: label for new point
    :param points: dictionary mapping labels to coordinates
    :param tree: adjListMap mapping label to neighbor labels
    :param OBSedgeGroup: obstacles
    :return: updated points and tree
    """
    newPoints = copy.deepcopy(points)
    numPoints = len(newPoints)

    lastIndex  = len(points)
    originPoints = copy.deepcopy(points)
    originTree = copy.deepcopy(tree)

    point = points[new_point]

    if not tree: # Checks if tree is empty
        tree[new_point] = [] # Add point as starting node (has no neighbors to start off)
        return newPoints, tree

    # Find closest point and edge
    closest_point, closest_pt_dist = find_closest_point(point, newPoints, tree)
    closest_edge, closest_edge_dist, closest_intersect_pt = find_closest_edge(new_point, closest_point, newPoints, tree)

    # If point, then add as neighbor to the closest point
    if closest_pt_dist <= closest_edge_dist:
        tree[closest_point].append(new_point)
        tree[new_point] = [closest_point]

    # If edge, then create a new point and update tree and newPoints
    else:
        # Add intersection point coordinates to points
        intersect_label = numPoints + 1
        newPoints[intersect_label] = closest_intersect_pt
        numPoints += 1

        # Update endpoint neighbors and add adjList for intersection and new points
        endpt_a = closest_edge[0]
        endpt_b = closest_edge[1]

        # Check collision here, if collide, then do not add point, return original tree
        # If no collision, add new edge and intersect point

        tree[endpt_a].remove(endpt_b)
        tree[endpt_a].append(intersect_label)

        tree[endpt_b].remove(endpt_a)
        tree[endpt_b].append(intersect_label)

        tree[new_point] = [intersect_label]

        tree[intersect_label] = [endpt_a, endpt_b, new_point]

    # # FOR TESTING, visualize tree after every step
    # displayRRTandPath(newPoints, tree, None)
    if tree.has_key(lastIndex):
        tmpEdgeIndexGroup = tree[lastIndex]
        anyEdgeIntersect = False
        pointA = newPoints[lastIndex]
        for edgeIndex in tmpEdgeIndexGroup:
            pointB = newPoints[edgeIndex]
            for obsedge in OBSedgeGroup:
                anyEdgeIntersect = anyEdgeIntersect or intersect(pointA, pointB, obsedge[0], obsedge[1])
            pointInterval = edgePointInterval(pointA, pointB)
            for point in pointInterval:
                if not isCollisionFree(robot, point, obstacles):
                    anyEdgeIntersect = True

    if anyEdgeIntersect == True:
        originPoints.pop(lastIndex,None)
        return originPoints,originTree
    else:
        return newPoints, tree

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):
    if not isCollisionFree(robot, startPoint, obstacles) or not isCollisionFree(robot, goalPoint, obstacles):
        print "collision start/goal point,no solution possible"
        exit(0)
    MAX_NODES = 120
    points = dict()
    tree = dict()
    path = []
    # Your code goes here.

    OBSedgeGroup=[]
    for obs in obstacles:
        i=0
        while i < len(obs):
            if i == len(obs)-1:
                OBSedgeGroup.append((obs[i],obs[0]))
            else:
                OBSedgeGroup.append((obs[i],obs[i+1]))
            i+=1
    #add boundary
    OBSedgeGroup.append(((0,0),(0,10)))
    OBSedgeGroup.append(((0,0),(10,0)))
    OBSedgeGroup.append(((10,0),(10,10)))
    OBSedgeGroup.append(((0,10),(10,10)))

    points[1]=startPoint

    nextPointIndex = 2

    X_BOUND = 10
    Y_BOUND = 10

    begin = timeit.default_timer()


    for i in range(MAX_NODES+1):
        if i == MAX_NODES:
            tmpPoint=goalPoint
        else:
            tmpPoint = randSamplePoint(X_BOUND,Y_BOUND)
        if isCollisionFree(robot,tmpPoint,obstacles):
            prevlen = len(points)
            prevTree = copy.deepcopy(tree)
            points[nextPointIndex]=tmpPoint
            points, tree = growComplexRRT(nextPointIndex, points,prevTree,OBSedgeGroup)
            postlen = len(points)
            nextPointIndex = nextPointIndex + postlen - prevlen
            print i
    stop = timeit.default_timer()
    print "running time: ",stop - begin
    goalLabel=1
    for pt in points:
        if points[pt] == goalPoint:
            goalLabel=pt
    path = basicSearch(tree, 1, goalLabel)

    return points, tree, path

if __name__ == "__main__":
    # # Retrive file name for input data
    # if(len(sys.argv) < 6):
    #     print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
    #     exit()
    #
    # filename = sys.argv[1]
    # x1 = float(sys.argv[2])
    # y1 = float(sys.argv[3])
    # x2 = float(sys.argv[4])
    # y2 = float(sys.argv[5])

    # FOR TESTING REMOVE LATER--------------------------------------------------------------------------
    filename = "robot_env_01.txt"
    x1 = 1.0
    y1 = 2.0
    x2 = 8.5
    y2 = 7
    #----------------------------------------------------------------------------------------------------

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
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))

    # Your visualization code
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)