#!/usr/bin/env python
from aenum import Enum
import rospy
import actionlib
import tf
import time
import operator
from copy import deepcopy
import numpy as np
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import get_name_of_constant
from smores_ros import action_client
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Empty
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

class pose_container(object):
    pose = None
    rot = None

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
        self.set_behavior_service = None

        self._ramp_place_pose = None
        self._ramp_place_rot = None
        self._ramp_pose = None
        self.nav_finished = False
        self.robot_yaw_adjust_finished = False
        self.region_robot = None
        self.region_second_drawer = None
        self.character = None
        self._first_drawer_pose = None
        self._first_drawer_rot = None
        self._second_drawer_pose = None
        self._second_drawer_rot = None
        self.first_drawer_open = False

        self._initialize()
        self.main()

    def onShutDown(self):
        rospy.loginfo("\n####################\n## Shutting down. ##\n####################")
        self.setBehavior("Arm", "stop", True)
        self._stopDriving()


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
        self.set_behavior_service = rospy.ServiceProxy(
                    self.param_dict["set_behavior_service_name"], set_behavior)
        self.cmd_vel_pub = rospy.Publisher(
                self.param_dict["drive_command_topic_name"], Twist, queue_size=1)
        self.rtabmap_service = rospy.ServiceProxy("/rtabmap/reset", Empty)
        self.rtabmap_odom_service = rospy.ServiceProxy("/rtabmap/reset_odom", Empty)

        rospy.Subscriber("/rtabmap/odom", Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.navigation_goal_status_cb, queue_size=1)
        #rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_cb, queue_size=1)

        rospy.on_shutdown(self.onShutDown)
        ### This is only for testing
        self.robot_pose = Pose()
        self.second_drawer_pose = Pose()
        # This is for in front of the drawer
        self.second_drawer_pose.position.x = 0.6
        self.second_drawer_pose.position.y = 0.0
        self.viewpoint_pose = Pose()
        self.viewpoint_pose.position.x = 0.2
        self.viewpoint_pose.position.y = 0.8
        self.viewpoint_pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -45.0/180*np.pi)
        self.viewpoint_pose.orientation.x = quat[0]
        self.viewpoint_pose.orientation.y = quat[1]
        self.viewpoint_pose.orientation.z = quat[2]
        self.viewpoint_pose.orientation.w = quat[3]

        self.robot_state = RobotState.GoToFirstDrawer


    def setBehavior(self, configuration_name, behavior_name, is_action=False):
        try:
            resp = self.set_behavior_service(configuration_name, behavior_name, is_action)
            return resp.status
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def resetMap(self, odom=False):
        try:
            if odom:
                self.rtabmap_odom_service()
            rospy.sleep(1)
            self.rtabmap_service()
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
        # If we saw ramp before, don't check again.
        # TODO maybe do this once a while
        if self._ramp_pose is not None:
            return
        for tag in data.detections:
            if tag.id == 0 and self.tf.frameExists("ramp_nav_goal"):
                # Now we need to find the tag pose in world frame
                pose, rot = self.getPositionFromTF("map", "ramp_nav_goal")
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

        # We need to divide the return value by 0.7 to match with the scale in behavior_planner
        return v/0.7, w * knob_w_over_v/0.7

    def aquireRamp(self):
        ''' This part is for block grabbing '''

        self.setBehavior("Arm", "drive", False)
        while not rospy.is_shutdown():
            self.rate.sleep()
            # Find tag_0
            pose, rot = self.getPositionFromTF(self.first_tag, "tag_0")
            if pose is None:
                rospy.loginfo("Cannot see tag_0")
                continue

            rospy.loginfo("Detect tgag_0 object at {} and {}.".format(pose, rot))

            #################################3
            # numpy arrays to 4x4 transform matrix
            trans_mat = tf.transformations.translation_matrix(pose)
            rot_mat = tf.transformations.quaternion_matrix(rot)
            # create a 4x4 matrix for tag_0_adjusted
            tag_0_adjusted_mat = np.dot(trans_mat, rot_mat) # adjusted tag0 obj wrt map, as a 4x4 matrix
            nav_goal_Trans_mat = tf.transformations.translation_matrix([-0.10, 0.0, -0.00])
            nav_goal_Rot_mat = tf.transformations.rotation_matrix(np.pi/4, [0.0,1.0,0.0])
            nav_goal_mat = np.dot(nav_goal_Trans_mat, nav_goal_Rot_mat)
            nav_goal_mat = np.dot(tag_0_adjusted_mat, nav_goal_mat)
            nav_goal_mat = tf.transformations.inverse_matrix(nav_goal_mat)
            move_pose = tf.transformations.translation_from_matrix(nav_goal_mat)
            move_rot = tf.transformations.quaternion_from_matrix(nav_goal_mat)

            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]
            rospy.loginfo("Pose is x: {:.4f}, y: {:.4f}, theta: {:.4f}".format(move_pose[0], move_pose[1], theta*180/np.pi))
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
                if theta > (threshold_degrees*np.pi/180) or theta < -(threshold_degrees*np.pi/180):
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

        self._stopDriving()

    def adjustRobotYaw(self, target_rot):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()
            rospy.loginfo("Waiting for Yaw adjustment to finish.")

            cmd = Twist()
            # Get robot rotation
            pose = deepcopy(self.robot_pose)

            rot = (pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w)
            theta = tf.transformations.euler_from_quaternion(rot, 'szyx')[0]
            rospy.logdebug("Target angle {}. Current robot angle {}.".format(target_rot, theta))

            if self.doVisualServo(None, theta, target_rot, 7.0/180*np.pi):
                self.robot_yaw_adjust_finished = True
                return True

        return False

    def adjustAlignment(self, tol, discrete = False):
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

            if abs(abs(theta) - np.pi/2) < tol:
                # We are at good alignment
                self.setBehavior("ShortSnake", "stop", True)
                return True
            else:
                # Need to adjust alignment
                if abs(theta) < np.pi/2:
                    # Turn cw
                    self.setBehavior("ShortSnake", "spinCWF", True)
                    if discrete:
                        rospy.sleep(0.05)
                        self.setBehavior("ShortSnake", "stop", True)
                        rospy.sleep(1.0)
                else:
                    # Turn ccw
                    self.setBehavior("ShortSnake", "spinCCWF", True)
                    if discrete:
                        rospy.sleep(0.5)
                        self.setBehavior("ShortSnake", "stop", True)
                        rospy.sleep(1.0)


    def adjustHeadTilt(self, configuration_name, move_tag, target_tag, target_value, do_abs, comp_func):
        _last_direction = ""
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                (move_pose, move_rot) = self.getPositionFromTF(target_tag,move_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(self.first_tag, e))
                continue

            if move_rot is None:
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
        if self.adjustHeadTilt("Arm","tag_1","usb_cam", 2.5, True, operator.gt):
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
                (move_pose, move_rot) = self.getPositionFromTF("tag_1", self.middle_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(self.middle_tag, e))
                continue

            if move_rot is None:
                continue
            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[1]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))

            # Check if the car is in position or not
            if move_pose[0] < -0.13:
                self.setBehavior("ShortSnake", "stop", True)
                rospy.loginfo("Arrived")

                # Prepare to spin in place
                self.setBehavior("ShortSnake", "preSpin", True)

                # Spin until it is aligned with drawer
                if self.adjustAlignment(tol = 30.0/180*np.pi, discrete = False):
                    # Sleep before check again
                    rospy.sleep(2)
                    if self.adjustAlignment(tol = 7.0/180*np.pi, discrete = True):
                        # Drive forward a bit
                        self.setBehavior("ShortSnake", "forward", True)
                        rospy.sleep(0.7)
                        self.setBehavior("ShortSnake", "stop", True)
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

    def getPositionFromTF(self, source, target):
        rospy.logdebug("Getting position for {!r} wrt to {!r}".format(target, source))
        if not self.tf.frameExists(target):
            rospy.logwarn("Cannot find frame {!r}".format(target))
            return None, None

        while not rospy.is_shutdown():
            try:
                pose, rot = self.tf.lookupTransform(source, target, rospy.Time(0))
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
        # We choose 0.5 speed to match with the scale in behavior_planner
        data = Twist()
        if y < target_rot - tol:
            # Turn left
            rospy.logdebug("Turning left")
            data.angular.z = 1.0
            self.cmd_vel_pub.publish(data)
            return False
        elif y > target_rot + tol:
            # Turn right
            rospy.logdebug("Turning right")
            data.angular.z = -1.0
            self.cmd_vel_pub.publish(data)
            return False
        return True

    def _stopDriving(self):
        data = Twist()
        for i in xrange(10):
            time.sleep(0.05)
            self.cmd_vel_pub.publish(data)
        self.setBehavior("", "", False)

    def doCharacterization(self):
        rospy.loginfo("Doing characterization ... ")
        rospy.sleep(3)
        while not rospy.is_shutdown():
            self.rate.sleep()
            # Now trigger characterization
            rospy.loginfo("Send character request.")
            self.character = self.getCharacter()

            # Find the region of current robot position and goal
            # We need to shift the robot pose forward a bit if the characterization is doing local view
            pose = deepcopy(self.robot_pose)
            # Move forward 40 cm
            dist = 0.4
            qot = [pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w]
            theta = tf.transformations.euler_from_quaternion(qot, 'szyx')[0]
            pose.position.x += dist * np.cos(theta)
            pose.position.y += dist * np.sin(theta)
            rospy.loginfo("Send region request for robot at {0.x} and {0.y}".format(pose.position))
            result = self.getRegionFromPose(pose)
            if result is None:
                rospy.logwarn("Cannot get region from pose. Skip")
                continue
            self.region_robot = result.region.data
            rospy.loginfo("The robot is in {}.".format(self.region_robot))
            rospy.loginfo("Send region request for second drawer at {0.x} and {0.y}".format(self.second_drawer_pose.position))
            result = self.getRegionFromPose(self.second_drawer_pose)
            if result is None:
                rospy.logwarn("Cannot get region from pose. Skip")
                continue
            self.region_second_drawer = result.region.data
            rospy.loginfo("The second drawer is in {}.".format(self.region_second_drawer))

            return

    def doTransformation(parent_mat, trans_v, rot_v):
        goal_tran = tf.transformations.translation_matrix(trans_v)
        goal_rot = tf.transformations.quaternion_from_matrix(rot_v)
        goal_mat = np.dot(goal_tran, goal_rot)
        goal_mat = np.dot(parent_mat, goal_mat)

        return goal_mat

    def main(self):
        # Switch camera view
        rospy.loginfo("Switching camera source...")
        pub = rospy.Publisher("april_source", Bool, queue_size = 1)
        for i in xrange(10):
            cmd = Bool()
            cmd.data = True
            pub.publish(cmd)
            rospy.sleep(0.1)

        # Turn on color tracking
        rospy.loginfo("Turning on color tracking...")
        pub = rospy.Publisher("color_switch", Bool, queue_size = 1)
        for i in xrange(10):
            cmd = Bool()
            cmd.data = True
            pub.publish(cmd)
            rospy.sleep(0.1)

        rospy.sleep(5)

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.robot_state == RobotState.GoToFirstDrawer:
                # First we need to see if the first drawer was detected before

                rospy.loginfo("Getting pink object location")
                pose, rot = self.getPositionFromTF("map", "pinkObj")
                if pose is None:
                    rospy.loginfo("Cannot see pink object")
                    continue

                rospy.loginfo("Detect pink object at {} and {}.".format(pose, rot))

                theta = tf.transformations.euler_from_quaternion(rot, 'sxyz')[2]
                z_quat = tf.transformations.quaternion_from_euler(theta + np.pi,0,0, 'szyx')
                #################################3
                # numpy arrays to 4x4 transform matrix
    		trans_mat = tf.transformations.translation_matrix(pose)
    		rot_mat = tf.transformations.quaternion_matrix(z_quat)
    		# create a 4x4 matrix for pinkObjAdjusted
    		pinkObjAdjusted_mat = np.dot(trans_mat, rot_mat) # adjusted pink obj wrt map, as a 4x4 matrix
                firstDrawerTrans_mat = tf.transformations.translation_matrix([-0.4, -0.1, 0.0])
                secondDrawerTrans_mat = tf.transformations.translation_matrix([0.2, -0.05, 0.0])
                firstDrawer_mat = np.dot(pinkObjAdjusted_mat, firstDrawerTrans_mat)
                secondDrawer_mat = np.dot(pinkObjAdjusted_mat, secondDrawerTrans_mat)
                ####################################

                self._first_drawer_pose = tf.transformations.translation_from_matrix(firstDrawer_mat)
                self._first_drawer_rot = tf.transformations.quaternion_from_matrix(firstDrawer_mat)
                self._second_drawer_pose = tf.transformations.translation_from_matrix(secondDrawer_mat)
                self._second_drawer_rot = tf.transformations.quaternion_from_matrix(secondDrawer_mat)

                self.first_drawer_pose = self.constructPose(self._first_drawer_pose, self._first_drawer_rot)
                self.second_drawer_pose = self.constructPose(self._second_drawer_pose, self._second_drawer_rot)
                self.tf_pub.sendTransform(self._first_drawer_pose , self._first_drawer_rot, rospy.Time.now(), "first_drawer", "map")
                self.tf_pub.sendTransform(self._second_drawer_pose , self._second_drawer_rot, rospy.Time.now(), "second_drawer", "map")

                # If the drawer if opened already, and the pink object is seen again, we move on to next step
                if self.first_drawer_open:
                    self.setRobotState(RobotState.GoToViewPoint)
                    continue

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
                    theta = tf.transformations.euler_from_quaternion(self._first_drawer_rot, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()
                        self.setBehavior("Arm", "stop", True)

                        # Now do the open action
                        self.setBehavior("Arm", "openDrawer", True)
                        self.setBehavior("Arm", "stand",True)



                # Reset the map
                rospy.loginfo("Reset map")
                self.resetMap(odom = True)
                rospy.sleep(5)
                self.first_drawer_open = True
                # Now we redo this part again without open the drawer to get a better pink object location


            if self.robot_state == RobotState.GoToViewPoint:

                # Turn off color tracking
                rospy.loginfo("Turning off color tracking...")
                pub = rospy.Publisher("color_switch", Bool, queue_size = 1)
                for i in xrange(10):
                    cmd = Bool()
                    cmd.data = False
                    pub.publish(cmd)
                    rospy.sleep(0.5)

                # First we need to see if the viewpoint was detected before
                if self.viewpoint_pose is None:
                    rospy.logerr("Viewpoint was not detected before.")
                    # TODO uncomment this
                    # return
                    continue

                # Let's do a initial turn first
                self.setBehavior("Arm", "drive", False)
                # Drive to point
                data = Twist()
                data.linear.x = -0.3
                for i in xrange(6):
                    self.cmd_vel_pub.publish(data)
                    rospy.sleep(0.5)
                data = Twist()
                data.angular.z = 1.0
                for i in xrange(13):
                    self.cmd_vel_pub.publish(data)
                    rospy.sleep(0.5)
                self._stopDriving()

                # Now we need to send the viewpoint position to navigation stack
                # Now let's send the goal and drive
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                for i in xrange(5):
                    self.sendNavGoalRequest(self.viewpoint_pose)
                    rospy.sleep(0.5)

                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    qot = [self.viewpoint_pose.orientation.x,
                           self.viewpoint_pose.orientation.y,
                           self.viewpoint_pose.orientation.z,
                           self.viewpoint_pose.orientation.w]
                    theta = tf.transformations.euler_from_quaternion(qot, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()
                        self.setBehavior("Arm", "stop", True)

                        # Reset rtab map
                        rospy.loginfo("Reset map")
                        self.resetMap()
                        rospy.sleep(5)
                        # Now do the character action
                        self.doCharacterization()

                        # Check if the robot is in the same region as the goal and put out an error
                        if self.region_robot == self.region_second_drawer:
                            rospy.logerr("The robot is in the same region and the second drawer.")
                        else:
                            candidates = self.character.poses.poses
                            rospy.loginfo("There are {} ramp candidates.".format(len(candidates)))
                            # Checking candidates to see if any of them connects the robot to the goal
                            matched_id = None
                            for i, pose in enumerate(candidates):
                                # Check if the robot and goal regions match with the candidate's regions
                                r1 = self.character.region1.data[i]
                                r2 = self.character.region2.data[i]
                                rospy.loginfo("Checking pose {0.x} {0.y} {0.z} that connects {1} {2}.".format(pose.position, r1, r2))

                                if (r1 == self.region_robot and r2 == self.region_second_drawer) \
                                    or (r2 == self.region_robot and r1 == self.region_second_drawer):
                                        rospy.loginfo("Match!")
                                        matched_id = i
                                        break
                                else:
                                        rospy.loginfo("Does not match!")

                            # If no candidate is matched
                            if matched_id is None:
                                rospy.logerr("Cannot find matching construction candidates. Quit")

                self.setRobotState(RobotState.GoToRamp)

            if self.robot_state == RobotState.GoToRamp:
                # The robot is driving to pick up the ramp


                # Find tag_0
                pose, rot = self.getPositionFromTF("map", "tag_0")
                if pose is None:
                    rospy.loginfo("Cannot see tag_0")
                    continue

                rospy.loginfo("Detect tgag_0 object at {} and {}.".format(pose, rot))

                theta = tf.transformations.euler_from_quaternion(rot, 'sxyz')[2]
                z_quat = tf.transformations.quaternion_from_euler(theta,0,0, 'szyx')
                #################################3
                # numpy arrays to 4x4 transform matrix
    		trans_mat = tf.transformations.translation_matrix([pose[0], pose[1], 0.0])
    		rot_mat = tf.transformations.quaternion_matrix(z_quat)
    		# create a 4x4 matrix for tag_0_adjusted
    		tag_0_adjusted_mat = np.dot(trans_mat, rot_mat) # adjusted tag0 obj wrt map, as a 4x4 matrix
                nav_goal_Trans_mat = tf.transformations.translation_matrix([-0.4, 0.0, 0.0])
                nav_goal_mat = np.dot(tag_0_adjusted_mat, nav_goal_Trans_mat)
                self._nav_goal_pose = tf.transformations.translation_from_matrix(nav_goal_mat)
                self._nav_goal_rot = tf.transformations.quaternion_from_matrix(nav_goal_mat)
                self.tf_pub.sendTransform(self._nav_goal_pose, self._nav_goal_rot, rospy.Time.now(), "ramp_nav_goal", "map")

                ####################################

                self._ramp_pose = self.constructPose(self._nav_goal_pose, self._nav_goal_rot)

                ## First we need to see if the ramp was detected before
                #if self._ramp_pose is None:
                #    rospy.logerr("Ramp was not detected before.")
                #    # TODO uncomment this
                #    # return
                #    continue

                # Now we need to send the ramp position to navigation stack
                # Now let's send the goal and drive
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                rng_rot = [self._ramp_pose.orientation.x,
                           self._ramp_pose.orientation.y,
                           self._ramp_pose.orientation.z,
                           self._ramp_pose.orientation.w]
                theta = tf.transformations.euler_from_quaternion(rng_rot, 'szyx')[0]
                z_quat = tf.transformations.quaternion_from_euler(theta,0,0, 'szyx')

                # pack into a pose object:
                self._ramp_pose.orientation.x = z_quat[0]
                self._ramp_pose.orientation.y = z_quat[1]
                self._ramp_pose.orientation.z = z_quat[2]
                self._ramp_pose.orientation.w = z_quat[3]
                rospy.loginfo("Send nav goal to ramp. {}".format(self._ramp_pose))
                self.sendNavGoalRequest(self._ramp_pose)
                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    rot = (self._ramp_pose.orientation.x,
                           self._ramp_pose.orientation.y,
                           self._ramp_pose.orientation.z,
                           self._ramp_pose.orientation.w)
                    theta = tf.transformations.euler_from_quaternion(rot, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()

                        # Switch camera view
                        rospy.loginfo("Switching camera source...")
                        pub = rospy.Publisher("april_source", Bool, queue_size = 1)
                        for i in xrange(10):
                            cmd = Bool()
                            cmd.data = False
                            pub.publish(cmd)
                            rospy.sleep(0.5)

                        # Now do the pick up action
                        self.aquireRamp()
                        self.setRobotState(RobotState.GoToPlacePoint)

            if self.robot_state == RobotState.GoToPlacePoint:
                # The robot is driving to the point to place the ramp

                # Let's do a back up
                self.setBehavior("Arm", "drive", False)
                # Drive to point
                data = Twist()
                data.linear.x = -0.3
                for i in xrange(50):
                    self.cmd_vel_pub.publish(data)
                    rospy.sleep(0.5)
                self._stopDriving()

                # Check if the characterization is done before
                #if self.region_robot is None:
                self.doCharacterization()

                # Check if the robot is in the same region as the goal
                if self.region_robot == self.region_second_drawer:
                    rospy.loginfo("The robot is in the same region and the second drawer.")
                    # This should not really happen
                    rospy.logerr("Do not know what to do yet.")
                    return

                candidates = self.character.poses.poses
                rospy.loginfo("There are {} ramp candidates.".format(len(candidates)))
                # Checking candidates to see if any of them connects the robot to the goal
                matched_id = None
                for i, pose in enumerate(candidates):
                    # Check if the robot and goal regions match with the candidate's regions
                    r1 = self.character.region1.data[i]
                    r2 = self.character.region2.data[i]
                    rospy.loginfo("Checking pose {0.x} {0.y} {0.z} that connects {1} {2}.".format(pose.position, r1, r2))

                    if (r1 == self.region_robot and r2 == self.region_second_drawer) \
                        or (r2 == self.region_robot and r1 == self.region_second_drawer):
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

                # numpy arrays to 4x4 transform matrix
                pose = candidates[matched_id].position
                rot = candidates[matched_id].orientation
                p = [pose.x, pose.y, pose.z]
                r = [rot.x, rot.y, rot.z, rot.w]
    		trans_mat = tf.transformations.translation_matrix(p)
    		rot_mat = tf.transformations.quaternion_matrix(r)
    		ramp_pose_mat = np.dot(trans_mat, rot_mat) # ramp_pose wrt map, as a 4x4 matrix
                ramp_place_trans = tf.transformations.translation_matrix([-0.4, 0.0, 0.0])
                ramp_place_mat = np.dot(ramp_pose_mat,ramp_place_trans)

                self._ramp_place_pose = tf.transformations.translation_from_matrix(ramp_place_mat)
                self._ramp_place_rot = tf.transformations.quaternion_from_matrix(ramp_place_mat)
                # Now let's send the goal and drive
                target_pose = self.constructPose(self._ramp_place_pose, self._ramp_place_rot)
                rospy.loginfo("The ramp placement pose is {0.x},{0.y},{0.z}.".format(target_pose.position))
                self.nav_finished = False
                self.robot_yaw_adjust_finished = False
                self.sendNavGoalRequest(target_pose)
                self.setBehavior("Arm", "drive", False)

                # Check if the robot finished navigation
                if self.returnAfterNavFinished():
                    rospy.loginfo("Finished navigation.")
                    rospy.loginfo("Adjust robot Yaw")
                    theta = tf.transformations.euler_from_quaternion(self._ramp_place_rot, 'szyx')[0]
                    # Do visual servo
                    if self.adjustRobotYaw(theta):
                        rospy.loginfo("Finished adjust robot Yaw.")
                        self._stopDriving()

                        # Now do the rest
                        rospy.loginfo("Place the ramp and climb.")
                        self.PlaceAndClimb()

                        rospy.sleep(2)
                        self.setBehavior("Arm", "spinTower", True)
                        return



























