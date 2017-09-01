#!/usr/bin/env python

from smores_ros.srv import region_req, character_req, character_reqResponse
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import Pose, PoseArray
import rospy

def handle_region_req(req):
    print "Got a region req"
    if req.request.position.x == 1:
        return Int32(1)
    else:
        return Int32(2)

def handle_character_req(req):
    print "Got a character req"
    p1 = Pose()
    p2 = Pose()
    p1.position.x = 1
    p2.position.x = 2
    p1.orientation.w = 1
    p2.orientation.w = 1
    pa = PoseArray()
    pa.poses = [p1, p2]

    resp = character_reqResponse()
    resp.poses = pa
    resp.region1.data = [1,2]
    resp.region2.data = [2,3]

    return resp

def mock_server():
    rospy.init_node('mock_server')
    rospy.Service('region_req', region_req, handle_region_req)
    rospy.Service('character_req', character_req, handle_character_req)
    print "Ready to mock."
    rospy.spin()

if __name__ == "__main__":
    mock_server()
