#!/usr/bin/env python
from math import pi
from aenum import Enum

import rospy
from smores_ros.data_file_loader import DataFileLoader
from smores_library.SmoresModule import SmoresCluster
from smores_library import MissionPlayer
from smores_library.name_map import *
from smores_ros.srv import set_behavior

class RobotConfiguration(Enum):
    Tank = 0
    Proboscis = 1

class BehaviorPlanner(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.robot_configuration = None
        self.set_robot_conf_sub = None

        self._initialize()
        self.main()

    def _initialize(self):
        self.param_name_list = ["~set_behavior_service_name",
                                ]
        self.robot_configuration = 0

        self._getROSParam()

        # Setup service
        rospy.Service(self.param_dict["set_behavior_service_name"],
                set_behavior, self.handle_set_behavior)

    def handle_set_behavior(self, request):
        rospy.loginfo("Received request {} and {}".format(request.configuration_name, request.behavior_name))
        return True

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name.lstrip("~")] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))

    def main(self):
        rospy.spin()
