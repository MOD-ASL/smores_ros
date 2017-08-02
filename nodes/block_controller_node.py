#!/usr/bin/env python
import rospy
from smores_ros.block_controller import BlockController

rospy.init_node('SMORES_Block_Controller', anonymous=True, log_level=rospy.DEBUG)
BC = BlockController()
