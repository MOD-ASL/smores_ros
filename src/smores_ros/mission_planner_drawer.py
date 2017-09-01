#!/usr/bin/env python
from aenum import Enum
import rospy
import actionlib
import tf
import time
from threading import Timer
import operator
from copy import deepcopy
import numpy as np
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import get_name_of_constant
from smores_ros import action_client
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Vector3, Pose, Twist, TransformStamped, PoseStamped
from smores_ros.srv import nbv_request, target_req, set_behavior, region_req, character_req
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

class RobotState(Enum):
    Idle = 0
    GoToRamp = 1
    GoToViewPoint = 2
    GoToFirstDrawer = 3
    GoToPlacePoint = 4
    Explore = 9

GoalStatus.to_string = classmethod(get_name_of_constant)

class MissionPlanner(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.nav_action_client = None
        self.cmd_vel_pub = None
        self.robot_state = None
        self.tf = None
        self.color_cache = {"pink":None, "green":None}
        self.robot_pose = None
        self.first_drawer_pose = None
        self.second_drawer_pose = None
        self.viewpoint_pose = None
        self.tf_pub = None

        self._ramp_place_pose = None
        self._ramp_place_rot = None
        self._ramp_pose = None
        self.nav_finished = False
        self.robot_yaw_adjust_finished = False

        self._initialize()
        self.main()

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name.lstrip("~")] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))

    def _initialize(self):
        self.param_name_list = ["~nbv_service_name",
                                "~region_req_service_name",
                                "~character_req_service_name",
                                "~dock_point_service_name",
                                "~pink_obj_topic_name",
                                "~navigation_action_name",
                                "~set_behavior_service_name",
                                "~drive_command_topic_name",
                                ]
        self.robot_state = RobotState.Idle
        self.tf = tf.TransformListener()
        self.tf_pub = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)
        self.first_tag = "tag_5"
        self.middle_tag = "tag_6"
        self.last_tag = "tag_7"

        self._getROSParam()

        # Setup client, publishers and subscribers
        self.nav_action_client = action_client.SimpleActionClient(
                self.param_dict["navigation_action_name"], MoveBaseAction)

        rospy.Subscriber("/rtabmap/odom", Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.navigation_goal_status_cb, queue_size=1)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_cb, queue_size=1)

        ### This is only for testing
        self.robot_pose = Pose()
        self.first_drawer_pose = Pose()
        self.first_drawer_pose.position.x = 1
        self.second_drawer_pose = Pose()
        self.second_drawer_pose.position.x = 1
        self.robot_state = RobotState.GoToRamp

    def setBehavior(self, configuration_name, behavior_name, is_action=False):
        try:
            set_behavior_service = rospy.ServiceProxy(
                    self.param_dict["set_behavior_service_name"], set_behavior)
            resp = set_behavior_service(configuration_name, behavior_name, is_action)
            return resp.status
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def setRobotState(self, new_state):
        rospy.loginfo("Switching robot state from {} to {}."
                .format(self.robot_state, new_state))
        self.robot_state = new_state

    def sendNavGoalRequest(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Sending navigation goal request.")
        self.nav_action_client.send_goal(goal)

    def tag_cb(self, data):
        for tag in data.detections:
            if tag.id == 0:
                # Now we need to find the tag pose in world frame
                pose, rot = self.getPositionFromTF("ramp_nav_goal", "map")
                if pose is not None:
                    self._ramp_pose = self.constructPose(pose, rot)

    def navigation_goal_status_cb(self, data):
        if len(data.status_list) == 0:
            self.nav_finished= False
        else:
            self.nav_finished= (data.status_list[-1].status == 3)

    def pose_cb(self, pose):
        self.robot_pose = pose.pose.pose

    def getRegionFromPose(self, pose):
        try:
            region = rospy.ServiceProxy(
                        self.param_dict["region_req_service_name"],
                        region_req)
            region_id = region(pose)
            return region_id
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))
            return None

    def returnAfterNavFinished(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            state = self.nav_action_client.get_state()
            rospy.loginfo("Waiting for navigation to finish. Getting status {}.".format(state))
            if self.nav_finished:
                return True
        return False

    def navigateToRamp(self):

        # drive until goal reached, then ask to continue:
        while not rospy.is_shutdown():
            if (not self.arrived) or (self.goal_rot is None):
                time.sleep(1)
                continue

            rospy.loginfo("Arrived ")
            theta = tf.transformations.euler_from_quaternion(self.goal_rot, 'szyx')[0]
            # Do visual servo
            if self.adjustRobotYaw(theta):
                self.goal_rot = None
                self.arrived = False
                # Stop
                rospy.loginfo("Stop at good angle")
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                self.cmd_vel_pub.publish(cmd)
                self.cmd_vel_pub.publish(cmd)

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
        # knob_distance_over_angle = 13.0  # from beginning of august
        knob_distance_over_angle = 20.0
        #knob_w_over_v = 20.0 # from beginning of august
        knob_w_over_v = 25.0
        K_distance = knob_distance_over_angle/(knob_distance_over_angle + 1.0)
        K_angle = 1.0/(knob_distance_over_angle + 1.0)

        # Robot turning speed
        w = -K_distance * d - K_angle * theta

        return v, w * knob_w_over_v

    def aquireRamp(self):
        ''' This part is for block grabbing '''

        self.setBehavior("Arm", "drive", False)
        while not rospy.is_shutdown():
           self.rate.sleep()
           try:
               (move_pose, move_rot) = self.getPositionFromTF(self.first_tag, "tag_0_goal")
           except TypeError as e:
               rospy.logerr("Cannot find position for {!r}: {}".format(self.first_tag, e))
               continue

           theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]
           rospy.loginfo("Pose is x: {:.4f}, y: {:.4f}, theta: {:.4f}".format(move_pose[0], move_pose[1], theta*180/pi))
           v, w = self.lineFollowController(move_pose[1], theta)
           rospy.loginfo("Velocity is v: {:.4f}, w: {:.4f}".format(v, w))

           if move_pose[0] < -0.025:  # drive until close enogh to block
               # Drive to point
               data = Twist()
               data.angular.z = w
               data.linear.x = v
               self.cmd_vel_pub.publish(data)
           else:
               rospy.loginfo("Visual servoing")
               # visual servo to pick up block
               threshold_degrees = 0.5
               if theta > (threshold_degrees*pi/180) or theta < -(threshold_degrees*pi/180):
                   self.doVisualServo(move_pose[0], theta)
               else:
                   rospy.logerr("stopping.")
                   self._stopDriving()
                   # Execute the pickup behavior:
                   rospy.loginfo("Picking up")
                   time.sleep(1)
                   self.setBehavior("", "", False)
                   self.setBehavior("Arm", "pickUp", True)
                   time.sleep(5)
                   self.setBehavior("", "", False)
                   return

        data = Twist()
        data.angular.z = 0.0
        data.linear.x = 0.0
        for i in xrange(10):
           self.cmd_vel_pub.publish(data)
           time.sleep(0.05)
        self.setBehavior("Arm", "", False)

    def adjustRobotYaw(self, target_rot):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()
            rospy.loginfo("Waiting for Yaw adjustment to finish.")

            cmd = Twist()
            # Get robot rotation
            pose = self.robot_pose

            rot = (pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w)
            theta = tf.transformations.euler_from_quaternion(rot, 'szyx')[0]
            rospy.logdebug("Target angle {}. Current robot angle {}.".format(target_rot, theta))

            if self.doVisualServo(None, theta, target_rot, 3.0/180*np.pi):
                self.robot_yaw_adjust_finished = True
                return True

        return False

    def adjustAlignment(self, highspeed=True):
        _last_direction = ""
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getPositionFromTF(self.last_tag, "tag_1")
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

    def adjustHeadTilt(self, configuration_name, move_tag, target_tag, target_value, do_abs, comp_func):
        _last_direction = ""
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getPositionFromTF(move_tag, target_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(self.first_tag, e))
                continue

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[2]
            if do_abs:
                theta = abs(theta)
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))

            if abs(theta - target_value) < 0.01:
                # We are at good tilt
                self.setBehavior(configuration_name, "stop", True)
                return True
            else:
                # Need to adjust tilt
                if comp_func(theta, target_value):
                    # Turn down
                    self.setBehavior(configuration_name, "adjustHeadTiltDOWN", True)
                else:
                    # Turn up
                    self.setBehavior(configuration_name, "adjustHeadTiltUP", True)

    def PlaceAndClimb(self):
        # Set configuration
        self.setBehavior("Arm", "", True)
        # Adjust ramp tilt
        if self.adjustHeadTilt("Arm","tag_1", "usb_cam", 2.45, True, operator.gt):
            pass
        else:
            rospy.logerr("Cannot adjust ramp tilt")
        self.setBehavior("Arm", "forward", True)
        time.sleep(9)
        # Stop
        self.setBehavior("Arm", "stop", True)
        # Lift the front face
        self.setBehavior("Arm", "breakRamp", True)
        # Start to climb
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
                (move_pose, move_rot) = self.getPositionFromTF(self.middle_tag, "tag_1")
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
                        if self.adjustHeadTilt("ShortSnake",self.first_tag, "tag_1", 0.1, False, operator.lt):
                            self.setBehavior("ShortSnake", "openDrawer", True)
                        else:
                            rospy.logerr("Failed to adjust heading")
                        return
                    else:
                        rospy.logerr("Failed to adjust alignment")
                else:
                    rospy.logerr("Failed to adjust alignment")


    def getCharacter(self):
        try:
            character = rospy.ServiceProxy(
                           self.param_dict["character_req_service_name"],
                           character_req)
            result = character(0)
            return result
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))
            return None

    def getPositionFromTF(self, child, parent, delay = False):

        rospy.logdebug("Getting position for {!r} wrt to {!r}".format(child, parent))
        if not self.tf.frameExists(child):
            rospy.logwarn("Cannot find frame {!r}".format(child))
            return None, None

        while not rospy.is_shutdown():
            try:
                pose, rot = self.tf.lookupTransform(child, parent, rospy.Time(0))
                if delay:
                    self._ramp_place_pose = pose
                    self._ramp_place_rot = rot
                return pose, rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return None, None

    def constructPose(self, pose, rot):
        target_pose = Pose()
        target_pose.position.x = pose[0]
        target_pose.position.y = pose[1]
        target_pose.position.z = pose[2]
        target_pose.orientation.x = rot[0]
        target_pose.orientation.y = rot[1]
        target_pose.orientation.z = rot[2]
        target_pose.orientation.w = rot[3]
        return target_pose

    def doVisualServo(self, x, y, target_rot = 0.0, tol = 0.01):
        data = Twist()
        if y < target_rot - tol:
            # Turn left
            rospy.logdebug("Turning left")
            data.angular.z = 0.35
            self.cmd_vel_pub.publish(data)
            return False
        elif y > target_rot + tol:
            # Turn right
            rospy.logdebug("Turning right")
            data.angular.z = -0.35
            self.cmd_vel_pub.publish(data)
            return False
        return True

    def _stopDriving(self):
        data = Twist()
        self.cmd_vel_pub.publish(data)
        self.cmd_vel_pub.publish(data)
        self.cmd_vel_pub.publish(data)
        self.cmd_vel_pub.publish(data)
        self.setBehavior("", "", False)

    def main(self):

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.robot_state == RobotState.GoToFirstDrawer:
                # First we need to see if the first drawer was detected before
                if self.first_drawer_pose is None:
                    rospy.logerr("First drawer was not detected before.")
                    # TODO uncomment this
                    # return
                    continue
                # Now we need to send the first drawer position to navigation stack
                # Now let's send the goal and drive
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                self.sendNavGoalRequest(self.first_drawer_pose)
                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    theta = tf.transformations.euler_from_quaternion(self.first_drawer_pose.orientation, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()
                        self.setBehavior("Arm", "stop", True)

                        # Now do the open action
                        self.setBehavior("Arm", "openFirstDrawer", True)

                self.setRobotState(RobotState.GoToViewPoint)

            if self.robot_state == RobotState.GoToViewPoint:
                # First we need to see if the viewpoint was detected before
                if self.viewpoint_pose is None:
                    rospy.logerr("Viewpoint was not detected before.")
                    # TODO uncomment this
                    # return
                    continue
                # Now we need to send the viewpoint position to navigation stack
                # Now let's send the goal and drive
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                self.sendNavGoalRequest(self.viewpoint_pose)
                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    theta = tf.transformations.euler_from_quaternion(self.viewpoint_pose.orientation, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()
                        self.setBehavior("Arm", "stop", True)

                        # Now do the character action
                        rospy.sleep(10)

                self.setRobotState(RobotState.GoToRamp)

            if self.robot_state == RobotState.GoToRamp:
                # The robot is driving to pick up the ramp

                # First we need to see if the ramp was detected before
                if self._ramp_pose is None:
                    rospy.logerr("Ramp was not detected before.")
                    # TODO uncomment this
                    # return
                    continue
                # Now we need to send the ramp position to navigation stack
                # Now let's send the goal and drive
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                self.sendNavGoalRequest(self._ramp_pose)
                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    theta = tf.transformations.euler_from_quaternion(self._ramp_pose.orientation, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()
                        self.setBehavior("Arm", "stop", True)

                        # Now do the pick up action
                        self.aquireRamp()
                        self.setRobotState(RobotState.GoToPlacePoint)

            if self.robot_state == RobotState.GoToPlacePoint:
                # The robot is driving to the point to place the ramp

                # We need to figure out the placement point first
                # Find the region of current robot position and goal
                rospy.loginfo("Send region request for robot.")
                region_robot = self.getRegionFromPose(self.robot_pose).region.data
                rospy.loginfo("The robot is in {}.".format(region_robot))
                rospy.loginfo("Send region request for second drawer.")
                region_second_drawer = self.getRegionFromPose(self.second_drawer_pose).region.data
                rospy.loginfo("The second drawer is in {}.".format(region_second_drawer))

                # Check if the robot is in the same region as the goal
                if region_robot == region_second_drawer:
                    # Move to the goal
                    rospy.loginfo("The robot is in the same region and the second drawer.")
                    # This should not really happen
                    rospy.logerr("Do not know what to do yet.")
                    return

                # Now get all characterization infomation
                rospy.loginfo("Send character request.")
                character = self.getCharacter()

                candidates = character.poses.poses
                rospy.loginfo("There are {} ramp candidates.".format(len(candidates)))
                # Checking check candidates to see if any of them connects the robot to the goal
                matched_id = None
                for i, pose in enumerate(candidates):
                    # Check if the robot and goal regions match with the candidate's regions
                    r1 = character.region1.data[i]
                    r2 = character.region2.data[i]
                    rospy.loginfo("Checking pose {0.x} {0.y} {0.z} that connects {1} {2}.".format(pose.position, r1, r2))

                    if (r1 == region_robot and r2 == region_second_drawer) \
                        or (r2 == region_robot and r1 == region_second_drawer):
                            rospy.loginfo("Match!")
                            matched_id = i
                            break
                    else:
                            rospy.loginfo("Does not match!")

                # If no candidate is matched
                if matched_id is None:
                    rospy.logerr("Cannot find matching construction candidates. Quit")
                    return

                # We get a match

                # Figure out the driving target position from the candidate position
                # Basically a position along the negative direction from the candidate position

                # Get the the ramp placement pose
                # We need to use a timer to get the transform and publish the transform during this time
                t = Timer(2, self.getPositionFromTF, ["ramp_place_pose", "map", True])
                t.start()

                # First let's publish the candidate pose
                for i in xrange(10):
                    pose = (candidates[matched_id].position.x,
                            candidates[matched_id].position.y,
                            candidates[matched_id].position.z)
                    rot = (candidates[matched_id].orientation.x,
                           candidates[matched_id].orientation.y,
                           candidates[matched_id].orientation.z,
                           candidates[matched_id].orientation.w)
                    self.tf_pub.sendTransform(pose, rot, rospy.Time.now(), "ramp_pose", "map")
                    # Now translate along the negative x direction
                    # TODO check this
                    pose = (-0.1, 0.0, 0.0)
                    self.tf_pub.sendTransform(pose, rot, rospy.Time.now(), "ramp_place_pose", "ramp_pose")
                    rospy.sleep(0.5)

                # Now let's send the goal and drive
                target_pose = self.constructPose(self._ramp_place_pose, self._ramp_place_rot)
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                self.sendNavGoalRequest(target_pose)
                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    theta = tf.transformations.euler_from_quaternion(target_pose.orientation, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()
                        self.setBehavior("Arm", "stop", True)

                        # Now do the rest
                        self.PlaceAndClimb()



























