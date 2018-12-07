#!/usr/bin/env python
import rospy
import rdt2_guiless as rdt2
from rdt.srv import *
from std_srvs.srv import Empty
from map_msgs.msg import *
from nav_msgs.msg import OccupancyGrid

grid = OccupancyGrid()
lastPath = None
usedPath = 0
clear_cost = rospy.ServiceProxy('/move_base/clear_costmaps', Empty())

#For Occupancy Grid
def changeCost(grid, x, y):
    grid.data

def handle_make_plan(req):
    global grid
    global lastPath
    global usedPath
    print("Making an evil plan...")
    #print(req.start, req.goal)
    #check if lastPath still valid
    '''
    if lastPath == None:
        lastPathIsValid = False
    else:
        if len(lastPath) == 2:
            lastPathIsValid = False
        else:
            lastPathIsValid = rdt2.validPath(lastPath, grid)
    lastPathSize = -1
    if lastPathIsValid:
        lastPathSize = rdt2.pathSize(lastPath)
    '''
    path = rdt2.main(req.start, req.goal, grid)
    if path == False:
        path = [req.start]
        #clear_cost()
    return make_planResponse(path)
    
    '''
    else:
        pathSize = rdt2.pathSize(path)
        if lastPathSize == -1 or usedPath == 10 or  pathSize < lastPathSize:
            print('new path')
            lastPath = path
            usedPath = 0
            return make_planResponse(path)
        else:
            usedPath +=1
            return make_planResponse(lastPath)
    return make_planResponse(path)
    '''

def handle_costmap_update(data):
    global grid
    #FIXME check header for time
    griddata=list(grid.data)
    i = 0
    y = data.y
    while y < (data.y + data.height):
        x = data.x
        while x < (data.x + data.width):
            griddata[y * grid.info.width + x] = data.data[i]
            i +=1
            x +=1
        y +=1
    grid.data = tuple(griddata)
    return
    
def handle_init(req):
    global grid
    grid = req.grid
    rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, handle_costmap_update) 
    #TODO Error checking
    return initResponse(True)
    
def rdt_server():
    rospy.init_node('rdt_server')
    s_init = rospy.Service('rdt_server_init', init, handle_init)
    s_make_plan = rospy.Service('rdt_server_make_plan', make_plan, handle_make_plan)
    print "services initiated"
    rospy.spin()

if __name__ == "__main__":
    rdt_server()
