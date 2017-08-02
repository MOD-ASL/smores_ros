#!/usr/bin/env python
import rospy
import tf
import time
import numpy as np
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Vector3, Pose, Twist
from smores_ros.srv import set_behavior

class BlockController(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.cmd_vel_pub = None
        self.tf = None

        self._initialize()
        self.main()

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name.lstrip("~")] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))

    def _initialize(self):
        self.param_name_list = ["~set_behavior_service_name",
                                "~drive_command_topic_name",
                                ]
        self._getROSParam()
        self.tf = tf.TransformListener()

        # Setup client, publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher(
                self.param_dict["drive_command_topic_name"], Twist, queue_size=1)

        # Waiting for all service to be ready
        rospy.loginfo("Waiting for set behavior service ...")
        rospy.wait_for_service(self.param_dict["set_behavior_service_name"])

    def main(self):


        # Move the shortsnake
        self.setBehavior("ShortSnake", "forward", True)
        # Wait for 2 seconds
        time.sleep(2)
        # Move the sensor box
        # This is blocking for 5 seconds
        self.setBehavior("Arm", "", True)
        self.setBehavior("Arm", "pushSensor", True)

        move_tag = "tag_2"
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                (move_pose, move_rot) = self.getTagPosition(move_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
                continue

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[1]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {}".format(move_pose[0], move_pose[1], theta))

                    return

        #move_tag = "tag_2"
        #rate = rospy.Rate(1)
        #time.sleep(10)
        #self.setBehavior("Arm", "drive", False)

        #back_up_counter = 0
        #_last_drive = False
        #while not rospy.is_shutdown():
        #    rate.sleep()
        #    try:
        #        (move_pose, move_rot) = self.getTagPosition(move_tag)
        #    except TypeError as e:
        #        rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
        #        continue

        #    theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[1]
        #    rospy.loginfo("Pose is {:.4f} and {:.4f} and {}".format(move_pose[0], move_pose[1], theta))

        #    if back_up_counter > 3:
        #        # Backup
        #        _last_drive = False
        #        data = Twist()
        #        data.linear.x = -0.1
        #        self.cmd_vel_pub.publish(data)
        #        time.sleep(3)
        #        back_up_counter = 0

        #    if move_pose[1] > 0.01 or move_pose[1] < -0.01:
        #        if _last_drive:
        #            _last_drive = False
        #            back_up_counter += 1
        #        self.doVisualServo(move_pose[0], move_pose[1])
        #    else:
        #        # Drive forward
        #        _last_drive = True
        #        data = Twist()
        #        data.linear.x = 0.1
        #        self.cmd_vel_pub.publish(data)
        #        if move_pose[0] < 0.03:
        #            rospy.logerr("Picking up")
        #            self.setBehavior("", "", False)
        #            self.setBehavior("Arm", "pickUp", True)
        #            time.sleep(5)
        #            self.setBehavior("", "", False)
        #            return

    def getTagPosition(self, tag_id):
        rospy.logdebug("Getting position for {!r}".format(tag_id))
        while not rospy.is_shutdown():
            try:
                return self.tf.lookupTransform("tag_4", tag_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return None

    def setBehavior(self, configuration_name, behavior_name, is_action=False):
        try:
            set_behavior_service = rospy.ServiceProxy(
                    self.param_dict["set_behavior_service_name"], set_behavior)
            resp = set_behavior_service(configuration_name, behavior_name, is_action)
            return resp.status
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def doVisualServo(self, x, y):
        data = Twist()
        if y > 0.01:
            # Turn left
            rospy.logdebug("Turning left")
            data.angular.z = 0.3
            self.cmd_vel_pub.publish(data)
        elif y < -0.01:
            # Turn right
            rospy.logdebug("Turning right")
            data.angular.z = -0.3
            self.cmd_vel_pub.publish(data)
