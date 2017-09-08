#!/usr/bin/env python
import rospy
from smores_ros.mission_planner_gap import MissionPlanner

rospy.init_node('SMORES_Mission_Planner', anonymous=True, log_level=rospy.INFO)
MP = MissionPlanner()
