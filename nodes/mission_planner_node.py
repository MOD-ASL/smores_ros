#!/usr/bin/env python
import rospy
from smores_ros.mission_planner_drawer import MissionPlanner

rospy.init_node('SMORES_Mission_Planner', anonymous=True, log_level=rospy.DEBUG)
MP = MissionPlanner()
