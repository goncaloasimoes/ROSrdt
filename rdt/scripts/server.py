!/usr/bin/env python

import rospy
from rdt.srv import *
import rdt2-guiless as rdt2

def handle_make_plan(req):
    print("Making an evil plan...")
    print(req.start, req.goal)
    path = rdt2.main() # FIXME, ADICIONAR COSTMAPS ETC
    return AddTwoIntsResponse(req.start, req.goal)

def handle_init(req):
    pass
    
def add_two_ints_server():
    rospy.init_node('rdt_server')
    s = rospy.Service('makeplan', make_plan, handle_make_plan)
    s = rospy.Service('init', init, handle_init)
    print "Ready to make evil plans..."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
