#!/usr/bin/env python
import rospy
import tf
import time
import sys
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
        self.rate = None
        self.first_tag = ""
        self.middle_tag = ""
        self.last_tag = ""


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
                                "~test_apriltags",
                                ]
        self._getROSParam()
        self.tf = tf.TransformListener()
        self.rate = rospy.Rate(10)
        self.first_tag = "tag_5"
        self.middle_tag = "tag_6"
        self.last_tag = "tag_7"


        if self.param_dict["test_apriltags"]:
            rospy.loginfo("Apriltag testing mode ...")
        else:
            # Setup client, publishers and subscribers
            self.cmd_vel_pub = rospy.Publisher(
                    self.param_dict["drive_command_topic_name"], Twist, queue_size=1)

            # Waiting for all service to be ready
            rospy.loginfo("Waiting for set behavior service ...")
            rospy.wait_for_service(self.param_dict["set_behavior_service_name"])

    def _test_apriltags(self):
        move_tag = self.first_tag
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getTagPosition(move_tag, "tag_0_goal")
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
                continue

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))
            v, w = self.lineFollowController(move_pose[1], theta)
            rospy.loginfo("Velocity is {:.4f} and {:.4f}".format(v, w))

            vel_l = v/0.2*70.0-w/0.4*25.0
            vel_r = -v/0.2*70.0-w/0.4*25.0
            rospy.loginfo("Wheel Velocity is {:.4f} and {:.4f}".format(vel_l, -vel_r))

    def adjustAlignment(self, highspeed=True):
        _last_direction = ""
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getTagPosition(self.last_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(self.last_tag, e))
                continue

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))

            if abs(abs(theta) - np.pi/2) < 0.01:
                # We are at good alignment
                self.setBehavior("ShortSnake", "stop", True)
                return True
            else:
                # Need to adjust alignment
                if abs(theta) < np.pi/2:
                    # Turn cw
                    if highspeed:
                        self.setBehavior("ShortSnake", "spinCWF", True)
                    else:
                        self.setBehavior("ShortSnake", "spinCWS", True)
                else:
                    # Turn ccw
                    if highspeed:
                        self.setBehavior("ShortSnake", "spinCCWF", True)
                    else:
                        self.setBehavior("ShortSnake", "spinCCWS", True)


    def adjustHeadTilt(self):
        _last_direction = ""
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getTagPosition(self.first_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(self.first_tag, e))
                continue

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[2]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))

            if abs(theta - 0.1) < 0.01:
                # We are at good tilt
                self.setBehavior("ShortSnake", "stop", True)
                return True
            else:
                # Need to adjust tilt
                if theta < 0.1:
                    # Turn down
                    if _last_direction == "down":
                        continue
                    else:
                        self.setBehavior("ShortSnake", "adjustHeadTiltDOWN", True)
                        _last_direction = "down"
                else:
                    # Turn up
                    if _last_direction == "up":
                        continue
                    else:
                        self.setBehavior("ShortSnake", "adjustHeadTiltUP", True)
                        _last_direction = "up"

    def main(self):
        if self.param_dict["test_apriltags"]:
            self._test_apriltags()

        if rospy.is_shutdown():
            return

        ## Set configuration
        #self.setBehavior("Arm", "", True)
        ## Lift the front face
        #self.setBehavior("Arm", "dropRamp", True)
        ## Break sensor box connection
        #self.setBehavior("Arm", "breakSensorBox", True)
        ## Stand up the shortsnake
        #self.setBehavior("ShortSnake", "", True)
        #self.setBehavior("ShortSnake", "stand", True)
        #time.sleep(1)
        ## Move the shortsnake
        #self.setBehavior("ShortSnake", "forward", True)
        ## Move the sensor box
        #self.setBehavior("Arm", "", True)
        #self.setBehavior("Arm", "pushSensor", True)
        ## Wait for 8 seconds
        #time.sleep(8)
        ## Stop the sensor box
        #self.setBehavior("Arm", "stop", True)

        # Set configuration
        self.setBehavior("Arm", "", True)
        # Lift the front face
        self.setBehavior("Arm", "breakRamp", True)
        # Move forward
        self.setBehavior("Arm", "climbRamp", True)
        # Wait
        time.sleep(1)
        # Stop the sensor box
        self.setBehavior("Arm", "stop", True)
        # Break sensor box connection
        self.setBehavior("Arm", "breakSensorBox", True)
        # Stand up the shortsnake
        self.setBehavior("ShortSnake", "", True)
        self.setBehavior("ShortSnake", "stand", True)
        time.sleep(1)

        # Resume climbing
        self.setBehavior("ShortSnake", "", True)
        self.setBehavior("ShortSnake", "forward", True)
        time.sleep(6)
        # Now bend shortsnake more
        self.setBehavior("ShortSnake", "bend", True)

        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getTagPosition(self.middle_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(self.middle_tag, e))
                continue

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[1]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))

            # Check if the car is in position or not
            if move_pose[0] < -0.15:
                self.setBehavior("ShortSnake", "stop", True)
                rospy.loginfo("Arrived")

                # Prepare to spin in place
                self.setBehavior("ShortSnake", "preSpin", True)

                # Spin until it is aligned with drawer
                if self.adjustAlignment(highspeed = True):
                    if self.adjustAlignment(highspeed = False):
                        if self.adjustHeadTilt():
                            self.setBehavior("ShortSnake", "openDrawer", True)
                        else:
                            rospy.logerr("Failed to adjust heading")
                        return
                    else:
                        rospy.logerr("Failed to adjust alignment")
                else:
                    rospy.logerr("Failed to adjust alignment")


        #self.setBehavior("Arm", "drive", False)

        #back_up_counter = 0
        #_last_drive = False
        #t = "goal"
        #while not rospy.is_shutdown():
        #    self.rate.sleep()
        #    try:
        #        (move_pose, move_rot) = self.getTagPosition(self.first_tag,"tag_0_goal")
        #    except TypeError as e:
        #        rospy.logerr("Cannot find position for {!r}: {}".format(self.first_tag, e))
        #        continue

        #    theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]
        #    rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))
        #    v, w = self.lineFollowController(move_pose[1], theta)
        #    rospy.loginfo("Velocity is {:.4f} and {:.4f}".format(v, w))

        #    #if back_up_counter > 3:
        #    #    # Backup
        #    #    _last_drive = False
        #    #    data = Twist()
        #    #    data.linear.x = -0.1
        #    #    self.cmd_vel_pub.publish(data)
        #    #    time.sleep(3)
        #    #    back_up_counter = 0

        #    if move_pose[0] < -0.03:
        #        # Drive to point
        #        data = Twist()
        #        data.angular.z = w
        #        data.linear.x = v
        #        self.cmd_vel_pub.publish(data)
        #    else:
        #        # Arrived at good point

        #        if theta > 0.01 or theta < -0.01:
        #            self.doVisualServo(move_pose[0], theta)
        #        else:
        #            # Drive forward
        #            data = Twist()
        #            data.linear.x = 0.05
        #            self.cmd_vel_pub.publish(data)
        #            if move_pose[0] > -0.007:
        #                rospy.logerr("Picking up")
        #                #self.setBehavior("", "", False)
        #                #self.setBehavior("Arm", "pickUp", True)
        #                #time.sleep(5)
        #                #self.setBehavior("", "", False)
        #                break

        #data = Twist()
        #data.angular.z = 0.0
        #data.linear.x = 0.0
        #for i in xrange(10):
        #    self.cmd_vel_pub.publish(data)
        #    time.sleep(0.05)
        #self.setBehavior("Arm", "", False)

    def getTagPosition(self, tag_id):
        rospy.logdebug("Getting position for {!r}".format(tag_id))
        while not rospy.is_shutdown():
            try:
                return self.tf.lookupTransform("tag_1",tag_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return None

    #def getTagPosition(self, move_tag_id, origin_tag_id):
    #    rospy.logdebug("Getting position for {!r} wrt to {!r}".format(move_tag_id, origin_tag_id))
    #    while not rospy.is_shutdown():
    #        try:
    #            return self.tf.lookupTransform(origin_tag_id, move_tag_id, rospy.Time(0))
    #        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #            rospy.logerr(e)
    #            return None

    def lineFollowController(self, d, theta):
        """
        A controller for following a line with a differential drive robot
        Assuming the line is y=0
        d: y coordinate of the robot  (meter)
        theta: is heading of the robot wrt the line (radian)
        """
        v = 0.12
        w = 0.0

        # Gain of the controller
        knob_distance_over_angle = 13.0
        knob_w_over_v = 20.0
        K_distance = knob_distance_over_angle/(knob_distance_over_angle + 1.0)
        K_angle = 1.0/(knob_distance_over_angle + 1.0)

        # Robot turning speed
        w = -K_distance * d - K_angle * theta

        return v, w * knob_w_over_v

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
        if y < -0.01:
            # Turn left
            rospy.logdebug("Turning left")
            data.angular.z = 0.3
            self.cmd_vel_pub.publish(data)
        elif y > 0.01:
            # Turn right
            rospy.logdebug("Turning right")
            data.angular.z = -0.3
            self.cmd_vel_pub.publish(data)
