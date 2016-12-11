#!/usr/bin/env python
import rospy
from smores_ros.behavior_planner import BehaviorPlanner

rospy.init_node('SMORES_Behavior_Planner', anonymous=True, log_level=rospy.DEBUG)
BP = BehaviorPlanner()
