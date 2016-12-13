#!/usr/bin/env python
import rospy
from smores_ros.reconfiguration_planner import ReconfigurationPlanner

rospy.init_node('SMORES_Reconfiguration_Planner', anonymous=True, log_level=rospy.DEBUG)
RP = ReconfigurationPlanner()
