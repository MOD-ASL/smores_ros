#!/usr/bin/env python
import rospy
import sensor_msgs
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, Float32
from geometry_msgs.msg import Twist
import time


#joystick output gets mapped to differntial drive of the whole configuration
class dd_joy:
    def __init__(self):

        self.lin_pos = 4
        self.ang_pos = 3

        rospy.init_node('joy_to_command', anonymous = True)
        rospy.Subscriber('/joy', Joy, self.callback)
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 1)

    def callback(self, data):
        linear = data.axes[self.lin_pos] * 0.2
        angular = data.axes[self.ang_pos] * 0.6
        print 'l: ' + str(linear) + ', r: ' + str(angular)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

#testing: works!
#config = smores_config(11)
#config.diff_drive(50, 0)
#time.sleep(1)
#config.diff_drive(0, 0)
#time.sleep(1)

mod = dd_joy()
rospy.spin()

