import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import copy

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
def basicSearch(tree, start, goal):
    path = []
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    
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
        neighbors = [x[0] for x in adjListMap[id]]

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

    # need also check if every edge intersects with polygons
    for edge in globalEdge:
        for obsEdge in edgeGroup:
            collision = collision or intersect(obsEdge[0], obsEdge[1], edge[0], edge[1])

    return not collision

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

    # test sentence (delete before final submission) with start position
    # point=(x1,y1)
    # print "robot at",point,"is collisionfree ?",isCollisionFree(robot,point,obstacles)
    ########################################


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



