import tf
import rospy
import time
import os
import yaml
from numpy import sqrt
from math import pi
from smores_ros import smores_controller
from smores_ros.srv import reconf_request
import rospkg
from std_msgs.msg import String

class ReconfigurationPlanner:
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []

        self._initialize()
        self.main()

    def _initialize(self):
        self.param_name_list = ["~reconf_request_service_name",
                                ]

        self._getROSParam()

        # Setup service and subscriber
        rospy.Service(self.param_dict["reconf_request_service_name"],
                reconf_request, self.handler_reconf_request)

    def handler_reconf_request(self, data):
        pass

    def main(self):
        pass

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name.lstrip("~")] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))
