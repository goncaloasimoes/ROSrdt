import math, random, numpy as np
import rospy
from scipy.ndimage import gaussian_filter1d
import tf
from math import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid


import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as patches


iterations = 3000
gamma = 1.0
search_space_dim = 100
#delta = lambda k: (gamma*(log(iterations - k)/iterations)**(1/search_space_dim))
delta = 0.05
CURR_IT = 1
threshold = 51


def getIndex(grid, x, y):
    #print('[%s, %s]' % (x,y))
    #world to map/grid (assume no rotation in map #FIXME)
    if x < grid.info.origin.position.x or y < grid.info.origin.position.y:
        return False

    mx = int(round((x - grid.info.origin.position.x) / grid.info.resolution))
    my = int(round((y - grid.info.origin.position.y) / grid.info.resolution))

    #if (mx < size_x_ && my < size_y_)
    #    return true;
    #grid to array index
    return my*grid.info.width + mx ##FIXME not sure if right

def getCostInGrid(grid, x, y):
    index = getIndex(grid, x, y)
    #print('index %s' % index)
    if not index or index > len(grid.data)-1 or index < 0:
        return 100
        #FIXME throw roserror
    #print('grid: %s ' % grid.data[index])
    return grid.data[index]

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

class Node:
    def __init__(self, position, parent, edgePoint=False):
        self.position = position
        self.parent = parent
        self.edgePoint = edgePoint
        if parent == None:
            self.cost = 0
            self.distRoot = 0
        else:
            self.cost = parent.cost + dist(position, parent.position)
            self.distRoot = parent.distRoot + 1
        self.children = []
    
    def add_child(self, node):
        self.children.append(node)

    def __repr__(self):
        return '[%s, %s]' % (self.position[0], self.position[1])

class Tree:
    def __init__(self, position):
        self.position = position
        self.allNodes = []
        self.rootNode = Node(position, None)
        self.allNodes.append(self.rootNode)
        self.nNodes = 1

    def add_edge(self, parent, newNode):
        # add Parent if edge point
        if parent.edgePoint:
            self.add_edge(parent.parent, parent)
        parent.add_child(newNode)
        self.allNodes.append(newNode)
        self.nNodes+=1

def equal_node(n1, n2):
    if n1.position == n2.position or np.isclose([dist(n1.position, n2.position)],[0], rtol=1e-03, atol=1e-05)[0]:
        return True
    return False

def bigger_tree(t1, t2):
    if t1.nNodes > t2.nNodes:
        return True
    return False

##FIXME remove
def plot_path(path, color):
    npPath = np.array(path)
    #fig,ax = plt.subplots(1)
    plt.plot(npPath[:,0], npPath[:,1], color=color, marker='o')

def plot_costmap(costmap):
    k = 0
    x = 0
    y = 0
    while k < len(costmap.data):
        if costmap.data[k] >= 90:
            plt.scatter([x],[y],color='green',marker='o')
        k+=1
        x+=1
        if x == costmap.info.width:
            x = 0
            y +=1

def plot_tree(tree,color):
    #print('!!%s' % tree.nNodes)
    plot_node(tree.allNodes[0],color)

def plot_node(node,color):
    i = 0
    while i < len(node.children):
        #print(i)
        plt.plot([node.children[i].position[0], node.position[0]], [node.children[i].position[1], node.position[1]], color=color, marker = '')
        plot_node(node.children[i],color)
        i +=1
    
### Get Path from initial position to goal
# node is only equal node between the two trees
# t1 is tree where node belongs too initially
# t2 is othe tree
def get_path(node1, t1, node2, t2, initial):
    if not initial:
        taux = t1
        t1 = t2
        t2 = taux
        nodeaux = node1
        node1 = node2
        node2 = nodeaux
    path1 = []
    path1.append(node1.position)
    node = node1.parent
    while node != None:
        path1.append(node.position)
        node = node.parent
    path1 = list(reversed(path1))
    path2 = []
    path2.append(node2.parent.position)
    node = node2.parent.parent
    while node != None:
        path2.append(node.position)
        node = node.parent
    #print('path2 %s' % path2)
    path = path1 + path2
    return path

def nearest_edge_point(node, alpha):
    #edge points
    Ax = node.position[0]
    Ay = node.position[1]
    Bx = node.parent.position[0]
    By = node.parent.position[1]
    #alpha
    Cx = alpha[0]
    Cy = alpha[1]
    t = ((Cx-Ax)*(Bx-Ax)+(Cy-Ay)*(By-Ay))/((Bx-Ax)**2+(By-Ay)**2)
    # new point
    D = [Ax+t*(Bx-Ax), Ay+t*(By-Ay)]
    ##check if new point D is between A and B, by checking if the distance AD or BD is bigger than distance AB
    distAB = dist(node.position, node.parent.position)
    distAD = dist(node.position, D)
    distBD = dist(node.parent.position, D)
    if distAD >= distAB or distBD >= distAB:
        return False
    return Node(D, node.parent, edgePoint=True)

def NEAREST(nodeList, alpha):
    bestPoint = nodeList[0]
    bestPointDist = dist(bestPoint.position, alpha) ##+ bestPoint.cost
    i = 1
    ## find best point
    while i < len(nodeList):
        newDist = dist(nodeList[i].position, alpha) ##+ nodeList[i].cost
        if newDist < bestPointDist:
            bestPoint = nodeList[i]
            bestPointDist = newDist
        i +=1
    #return bestPoint
    ## find nearest point in edges
    if len(nodeList) > 1:
        bestEdgePoint = nearest_edge_point(nodeList[1], alpha)
        if bestEdgePoint != False:
            bestEdgeDist = dist(bestEdgePoint.position, alpha)## + bestEdgePoint.cost
        else:
            bestEdgeDist = 1e18
        i = 2
        while i < len(nodeList):
            newEdgePoint = nearest_edge_point(nodeList[i], alpha)
            if newEdgePoint == False:
                i+=1
                continue
            newEdgeDist = dist(newEdgePoint.position, alpha)## + newEdgePoint.cost
            if bestEdgePoint == False or newEdgeDist < bestEdgeDist:
                bestEdgePoint = newEdgePoint
                bestEdgeDist = newEdgeDist
            i+=1
    else:
        return bestPoint
    #print('%s %s' % (bestPointDist, bestEdgeDist))
    # get closest
    if bestPointDist <= bestEdgeDist:
        #print('point')
        return bestPoint
    else:
        return bestEdgePoint

def COLISION(qn, new, costmap):#FIXME the algorithm iterator might need some adjustments
    x = qn[0]
    y = qn[1]
    x2 = new[0]
    y2 = new[1]
    w = x2 - x
    h = y2 - y
    dx1 = 0
    dy1 = 0
    dx2 = 0
    dy2 = 0
    if w<0:
        dx1 = -costmap.info.resolution #-1
    elif w>0: 
        dx1 = costmap.info.resolution #1
    if h<0:
        dy1 = -costmap.info.resolution #-1
    elif h>0:
        dy1 = costmap.info.resolution #1 
    if w<0:
        dx2 = -costmap.info.resolution #1
    elif w>0:
        dx2 = costmap.info.resolution #1
    longest = abs(w)
    shortest = abs(h)
    if not longest>shortest:
        longest = abs(h)
        shortest = abs(w)
        if h<0:
            dy2 = -costmap.info.resolution #1
        elif h>0:
            dy2 = costmap.info.resolution #1
        dx2 = 0            
    numerator = longest/2
    #skip qn (first point)
    bestPoint = [x,y] 
    numerator += shortest
    if not numerator<longest:
        numerator -= longest
        x += dx1
        y += dy1
    else:
        x += dx2
        y += dy2
    i = costmap.info.resolution
    #print('longest %s' % longest)
    while i <=longest + costmap.info.resolution:
        ##check if went over final point
        if i >= longest:
            if getCostInGrid(costmap, x2, y2) >= 81:
                break
            else:
                bestPoint = [x2,y2]
                break
        ##check point
        elif getCostInGrid(costmap, x, y) >= 81: # 50 aka 127 from http://wiki.ros.org/costmap_2d for definitely not in collision
            #print('break %s' % [x,y])
            break
        bestPoint = [x,y]
        #increment
        numerator += shortest
        if not numerator<longest:
            numerator -= longest
            x += dx1
            y += dy1
        else:
            x += dx2
            y += dy2 
        i +=costmap.info.resolution
    # print(qn)
    # print(new)
    #print('bestPoint %s' % bestPoint)
    # print()
    return bestPoint

def STOPPING_CONFIGURATION(qn, alpha, costmap):
    #print('-stopping %s' % alpha)
    averageQn = qn.position
    averageNode = alpha
    #averageQn = [int(round(qn.position[0])), int(round(qn.position[1]))]
    #averageNode = [int(round(alpha[0])), int(round(alpha[1]))]
    # averageQn == averageNode
    if (np.isclose(averageQn, averageNode,rtol=1e-03, atol=1e-05)[0] and np.isclose(averageQn, averageNode,rtol=1e-03, atol=1e-05)[1]):
        #print('qn')
        return qn

    noColisionPoint = COLISION(averageQn, averageNode, costmap)
    node = Node(noColisionPoint, qn)
    #print(qn)
    #print(node)
    if equal_node(node, Node(averageQn,None)):
        #print('equal\n')
        return qn
    if not equal_node(node, Node(averageNode,None)):
        alpha = noColisionPoint
    #return Node(alpha, qn)
    if dist(qn.position,alpha) < delta or np.isclose([dist(qn.position,alpha)],[delta])[0]:
        node = Node(alpha, qn)
    else:
        theta = atan2(alpha[1]-qn.position[1],alpha[0]-qn.position[0])
        node = Node([qn.position[0] + delta*cos(theta), qn.position[1] + delta*sin(theta)], qn)
    return node

    ##check colision using Bresenham's line algorithm
    # average positions to pixels
    

    ##FIXME remove this
    if node.position[0] < 16 and node.position[0] > 14 and node.position[1] < 19 and node.position[1] > 11:
        return qn
    return node

def rdt_balanced_bidirectional(pos, goal, iterations, alpha, costmap):
    Tinitial = Tree(pos)
    Tgoal = Tree(goal)
    initial = True # initial
    Tsmall = Tinitial
    Tbig = Tgoal
    print('start')
    print(pos)
    print(goal)

    #generate tree
    i = 0
    while i < iterations:
        #print('##Iter %s' % i)
        #print('Tsmall %s' % Tsmall.nNodes)
        #print('Tbig %s' % Tsmall.nNodes)
        #print()
        #print('')
        #print('alpha %s' % alpha[i])
        qn = NEAREST(Tsmall.allNodes, alpha[i])
        #print('nearest %s' % qn)
        qs = STOPPING_CONFIGURATION(qn, alpha[i], costmap)
        #print('stopping %s' % qs)
        if not equal_node(qn,qs):
            #Tsmall.add_vertex(qs):
            Tsmall.add_edge(qn,qs)
            
            qnB = NEAREST(Tbig.allNodes, qs.position)
            #print('nearestB %s' % qnB)
            qsB = STOPPING_CONFIGURATION(qnB, qs.position, costmap)
            #print('stoppingB %s' % qsB)
            if not equal_node(qnB,qsB):
                #Tbig.add_vertex(qsB)
                Tbig.add_edge(qnB,qsB)
                
            # Reached goal
            if equal_node(qsB,qs):
                #plt.figure()
                #plot_costmap(costmap)
                #plot_tree(Tinitial,'blue')
                #plot_tree(Tgoal,'red')

                # get solution
                path = get_path(qs, Tsmall, qsB, Tbig, initial)
                #plot_path(path,'green')
                #plt.show()
                # Aqui
                newpath = optimizePath(path, costmap)
                #return newpath
                #plt.figure()
                #plot_path(newpath,'red')
                #plot_tree(Tinitial,'blue')
                #plot_tree(Tgoal,'red')
                plt.show()

                return pose_stamped_path(newpath)
        if bigger_tree(Tsmall, Tbig):
            initial = not initial
            aux = Tsmall
            Tsmall = Tbig
            Tbig = aux
        i+=1
    #plt.figure()
    #plot_costmap(costmap)
    #plot_tree(Tinitial,'blue')
    #plot_tree(Tgoal,'red')
    #plt.show()
    # no solution found 
    #return FAILURE
    return False

def scipy_bspline(cv, n=100, degree=3, periodic=False):
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
    """
    cv = np.asarray(cv)
    count = cv.shape[0]
    x = cv[:,0]
    y = cv[:,1]
    t = np.linspace(0, 1, len(x))
    t2 = np.linspace(0, 1, 100)

    x2 = np.interp(t2, t, x)
    y2 = np.interp(t2, t, y)
    sigma = 10
    x3 = gaussian_filter1d(x2, sigma)
    y3 = gaussian_filter1d(y2, sigma)

    x4 = np.interp(t, t2, x3)
    y4 = np.interp(t, t2, y3)

    cv[:,0] = x4
    cv[:,1] = y4
    return cv.tolist()


def optimizePath(path, costmap):
    newpath = list(path)
    #print('newpath %s' % newpath)
    i = 0
    while i <= len(newpath) - 3 and len(newpath) >= 3:
        #print(i)
        j = i+2
        best = 0
        while j < len(newpath):
            destination = newpath[j]
            if equal_node(Node(COLISION(newpath[i], destination, costmap),None),Node(destination,None)):
                best = j
                #newpath.pop(i+1)
            j+=1
        if best != 0:
            j = i+1
            while j < best:
                newpath.pop(i+1)
                j +=1
        i+=1
    return newpath
    #curve_path = scipy_bspline(newpath,n=len(newpath)*2,degree=2,periodic=False)
    #print(curve_path)
    #return curve_path

def pose_stamped_path(path):
    pose_list = []
    #iterate path
    for i in range(len(path)):
        # create Point
        position = Point(float(path[i][0]), float(path[i][1]), 0)
        if i != (len(path)- 1):
            # euler angle between points
            deltaX = path[i+1][0] - path[i][0]
            deltaY = path[i+1][1] - path[i][1]
            euler_z = atan2(deltaY, deltaX) + math.pi
            # euler angles to quaternion
            quaternion = tf.transformations.quaternion_from_euler(0, 0, euler_z)
            # create Quaternion
            orientation = Quaternion(*quaternion)
        #else: in last position, used last used quaternion, so robot does not spin upon reaching goal
        # create Pose
        pose = Pose(position=position, orientation=orientation)
        # create Header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        # create PoseStamped for each tuple
        pose = PoseStamped(header = header, pose = pose)
        # add to list
        pose_list.append(pose)
    #FIXME
    #print('pose list %s' % pose_list)
    pose_list.pop()
    return pose_list

def validPath(path, costmap):
    print(path.__class__)
    index = 0
    while index < len(path)-1:
        start = [path[index].pose.position.x, path[index].pose.position.y]
        destination = [path[index+1].pose.position.x, path[index+1].pose.position.y]
        if not equal_node(Node(COLISION(start, destination, costmap),None), Node(destination,None)):
           return False
        index +=1
    return True

def pathSize(path):
    totaldist = 0
    index = 0
    while index < len(path)-1:
        start = [path[index].pose.position.x, path[index].pose.position.y]
        destination = [path[index+1].pose.position.x, path[index+1].pose.position.y]
        totaldist += dist(start, destination)
        index +=1
    return totaldist
    
def main(start, goal, grid):
    global iterations
    '''
    #FIXME
    grid = np.zeros((20,20))
    grid[0:18,13:16] = 50
    grid[0:10,6:10] = 50
    grid[12:14,0:11] = 50
    grid[17:18,2:13] = 50
    xdim = 20
    ydim = 20
    ## grid from occupancy grid
    costmap = np.array(grid).reshape((xdim,ydim))
    posX = 1
    posY = 1
    goalX = 19
    goalY = 19
    pos = [posX,posY]
    goal = [goalX,goalY]
    '''
    #find window of values different from unknown #-1#FIXME on grid
    minX = grid.info.width
    minY = grid.info.height
    maxX = 0
    maxY = 0
    k = 0
    x = 0
    y = 0
    while k < len(grid.data):
        if grid.data[k] != 0:#FIXME
            if x < minX:
                minX = x
            if x > maxX:
                maxX = x
            if y < minY:
                minY = y
            if y > maxY:
                maxY = y
        k+=1
        x+=1
        if x == grid.info.width:
            x = 0
            y +=1
    '''    
    j = 0
    while i < grid.info.height:
        i = 0
        while j < grid.info.width:
            if costmap[k] != 0:#FIXME
                if i < minX:
                    minX = i
                if i > maxX:
                    maxX = i
                if j < minY:
                    minY = j
                if j > maxY:
                    maxY = j
            k+=1
            j+=1
        i+=1
    '''
    #Window to generate samples
    #Xwindow = [minX,maxX]
    #Ywindow = [minY,maxY]
    ##
    #print(len(grid.data))
    #print(grid.info.width)
    #print(grid.info.height)
    ##
    Xwindow = [0,grid.info.width]
    Ywindow = [0,grid.info.height]
    #generate #iterations samples
    alpha = []
    i = 0
    while i < iterations:
        new = [random.uniform(Xwindow[0],Xwindow[1]),random.uniform(Ywindow[0],Ywindow[1])]
        wx = grid.info.origin.position.x + (new[0]+0.5)*grid.info.resolution
        wy = grid.info.origin.position.y + (new[1]+0.5)*grid.info.resolution
        alpha.append([wx,wy])
        i+=1
    #plt.figure()
    #plt.scatter(np.array(alpha)[:,0], np.array(alpha)[:,1], color='pink', marker = 'o')
    #fix threshold for stuck cases
    global threshold
    cost = getCostInGrid(grid,start.pose.position.x, start.pose.position.y)
    if cost > threshold:
        threshold = cost #+ 10
    threshold = 61
    print(threshold)
    #FIXME maybe start 
    path = rdt_balanced_bidirectional([start.pose.position.x, start.pose.position.y], [goal.pose.position.x, goal.pose.position.y], iterations, alpha, grid)
    if not path == False:
        path.append(goal)
        print(len(path))
    #print('final path %s' % path)
    return path

if __name__ == '__main__':
    main()
