#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Vector3
from smores_ros.srv import nbv_request, target_req

class MissionPlanner(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []

        self._initialize()
        self.main()

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))
                rospy.signal_shutdown("Cannot find parameter {}.".format(para_name))

    def _initialize(self):
        self.param_dictam_name_list = ["nbv_service_name",
                                       "dock_point_service_name",
                                       "pink_obj_topic_name",
                                       "blue_obj_topic_name",
                                ]
        self._getROSParam()
        print self.param_dict

        # Setup publishers and subscribers
        rospy.Subscriber(self.param_dict["pink_obj_topic_name"],
                         Vector3, self._pink_obj_cb, queue_size=1)

    def _pink_obj_cb(self, data):
        rospy.loginfo("Received Pink Object")

    def getNBVPose(self):
        rospy.wait_for_service(self.param_dict["nbv_service_name"])
        try:
            getNBV = rospy.ServiceProxy(
                    self.param_dict["nbv_service_name"], nbv_request)
            resp = getNBV(0)
            return resp.nbv_pose
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))
            rospy.signal_shutdown("Service call failed: {}".format(e))

    def getDockPose(self):
        rospy.wait_for_service(self.param_dict["nbv_service_name"])
        try:
            get_dock = rospy.ServiceProxy(
                    self.param_dict["nbv_service_name"], target_req)
            resp = get_dock(0)
            return resp.nav_point, resp.recon_type
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))
            rospy.signal_shutdown("Service call failed: {}".format(e))

    def main(self):
        pass

        # Do explore

        # Do dock
