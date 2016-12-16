#!/usr/bin/env python
from aenum import Enum
import rospy
import actionlib
import tf
import time
import numpy as np
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import get_name_of_constant
from smores_ros import action_client
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Vector3, Pose, Twist
from smores_ros.srv import nbv_request, target_req, set_behavior
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotState(Enum):
    Idle = 0
    Explore = 1
    DriveToDock = 2
    VisualServo = 3
    ReconfigurationT2P = 4
    ReconfigurationP2T = 5
    FetchPink = 6
    FindBlue = 7
    DriveToBlue = 8
    DropPink = 9
    Done = 10
    VisualServoForRepickup = 11

GoalStatus.to_string = classmethod(get_name_of_constant)

class MissionPlanner(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.nav_action_client = None
        self.reconf_signal_pub = None
        self.cmd_vel_pub = None
        self.robot_state = None
        self.tf = None
        self.color_cache = {"pink":None, "blue":None, "green":None}
        self._current_color = ""
        self._done_color = []
        self.reconf_type = None
        self._carry_item = False

        self._color_dock_time = None
        self._blue_dock_time = None

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
                                "~dock_point_service_name",
                                "~pink_obj_topic_name",
                                "~blue_obj_topic_name",
                                "~navigation_action_name",
                                "~reconf_signal_topic_name",
                                "~reconf_status_topic_name",
                                "~set_behavior_service_name",
                                "~drive_command_topic_name",
                                ]
        self.robot_state = RobotState.Idle
        self.tf = tf.TransformListener()

        self._getROSParam()

        # Setup client, publishers and subscribers
        self.nav_action_client = action_client.SimpleActionClient(
                self.param_dict["navigation_action_name"], MoveBaseAction)
        self.reconf_signal_pub = rospy.Publisher(
                self.param_dict["reconf_signal_topic_name"], String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(
                self.param_dict["drive_command_topic_name"], Twist, queue_size=1)

        # Waiting for all service to be ready
        rospy.loginfo("Waiting for nbv pose service ...")
        rospy.wait_for_service(self.param_dict["nbv_service_name"])
        rospy.loginfo("Waiting for dock point service ...")
        rospy.wait_for_service(self.param_dict["dock_point_service_name"])
        rospy.loginfo("Waiting for set behavior service ...")
        rospy.wait_for_service(self.param_dict["set_behavior_service_name"])
        rospy.loginfo("Waiting for navigation action service ...")
        self.nav_action_client.wait_for_server()

    def setRobotState(self, new_state):
        rospy.loginfo("Switching robot state from {} to {}."
                .format(self.robot_state, new_state))
        self.robot_state = new_state

    def isColorObjDetected(self, color):
        try:
            color_obj = rospy.wait_for_message(self.param_dict[color + "_obj_topic_name"],
                                              Vector3, timeout=1.0)
            pose = self.getColorObjPose(color)
            if pose is None:
                return False

            rospy.loginfo("{} object maybe detected. Checking.".format(color))
            rospy.sleep(5)
            pose = self.getColorObjPose(color)
            if pose is None:
                rospy.loginfo("False alarm.")
                return False
            else:
                rospy.loginfo("{} object definitely detected.".format(color))
                return True
        except (rospy.ROSException, AttributeError):
            return False

    def getColorObjPose(self, color, cached = False):
        try:
            pose, rot = self.tf.lookupTransform("map", color + "Obj", rospy.Time(0))
            self.color_cache[color] = pose
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            if cached:
                return self.color_cache[color]
            return None

    def getRobotPoseWRTColor(self, color):
        try:
            pose, rot = self.tf.lookupTransform("base_link", color + "Obj", rospy.Time(0))
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return None

    def getNBVPose(self):
        try:
            getNBV = rospy.ServiceProxy(
                    self.param_dict["nbv_service_name"], nbv_request)
            resp = getNBV(0)
            return resp.nbv_pose
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def getDockPose(self, target_pose):
        try:
            get_dock = rospy.ServiceProxy(
                    self.param_dict["dock_point_service_name"], target_req)
            data = Pose()
            data.position.x = target_pose[0]
            data.position.y = target_pose[1]
            data.position.z = target_pose[2]
            resp = get_dock(data)
            return resp.nav_point, resp.recon_type
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def setBehavior(self, configuration_name, behavior_name, is_action=False):
        try:
            set_behavior_service = rospy.ServiceProxy(
                    self.param_dict["set_behavior_service_name"], set_behavior)
            resp = set_behavior_service(configuration_name, behavior_name, is_action)
            return resp.status
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def sendNavGoalRequest(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Sending navigation goal request.")
        self.nav_action_client.send_goal(goal)

    def sendReconfSignal(self, reconf_direction):
        self.reconf_signal_pub.publish(reconf_direction)

    def isReconfFinished(self):
        try:
            rospy.wait_for_message(self.param_dict["reconf_status_topic_name"],
                                   String, timeout = 1.0)
            return True
        except rospy.ROSException:
            return False

    def doVisualServo(self, x):
        data = Twist()
        if x < 15.:
            # Turn left
            data.angular.z = 0.18
            self.cmd_vel_pub.publish(data)
        elif x > 25.:
            # Turn right
            data.angular.z = -0.18
            self.cmd_vel_pub.publish(data)

    def _stopDriving(self):
        data = Twist()
        data.angular.z = 0.0
        self.cmd_vel_pub.publish(data)
        self.cmd_vel_pub.publish(data)
        self.cmd_vel_pub.publish(data)
        self.setBehavior("", "", False)

    def _resumeDriving(self):
        self.setBehavior("Tank", "Tank_diff.xml", False)

    def main(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            rate.sleep()
            if self.robot_state == RobotState.DriveToDock:
                if self._color_dock_time is None:
                    self._color_dock_time = time.time()
                if time.time() - self._color_dock_time > 20.0:
                    rospy.loginfo("Getting an updated {} docking pose.".format(self._current_color))
                    color_pose = self.getColorObjPose(self._current_color)
                    if color_pose is None:
                        rospy.logwarn("Cannot find {} object. Roll back to Idle.".format(self._current_color))
                        self.setRobotState(RobotState.Idle)
                        self._color_dock_time = None
                        continue
                    dock_pose, self.reconf_type = self.getDockPose(color_pose)
                    self.sendNavGoalRequest(dock_pose)
                    self._color_dock_time = time.time()

                state = self.nav_action_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    # Arrived at docking point
                    # Do visual servo
                    #TODO: If no need for reconf, don't servo?
                    rospy.loginfo("Finished docking.")
                    self.setBehavior("", "", False)
                    if self._current_color == "pink":
                        self.setRobotState(RobotState.VisualServo)
                    elif self._current_color == "green":
                        self.setRobotState(RobotState.VisualServoForRepickup)

                elif state == GoalStatus.ACTIVE:
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                    self.isColorObjDetected("blue")
                    if self._current_color not in self._done_color:
                        if self._current_color == "pink":
                            self.isColorObjDetected("green")
                        elif self._current_color == "green":
                            self.isColorObjDetected("pink")
                else:
                    rospy.loginfo("Getting actionlib state {} when going to {} dock.".format(GoalStatus.to_string(state), self._current_color))

            if self.robot_state == RobotState.VisualServo or self.robot_state == RobotState.VisualServoForRepickup:
                self.setBehavior("Tank", "Tank_diff.xml", False)
                color = ""
                try:
                    if self._carry_item:
                        color = "blue"
                    else:
                        color = self._current_color

                    pose = rospy.wait_for_message(self.param_dict[color + "_obj_topic_name"],
                                                  Vector3, timeout=1.0)
                except rospy.ROSException:
                    pose = None
                    rospy.logwarn("Cannot find {} object when visual servoing. Random spinning.".format(color))
                    # Turn left
                    data = Twist()
                    data.angular.z = 0.3
                    self.cmd_vel_pub.publish(data)

                if pose is not None:
                    rospy.loginfo("Current x value is {}.".format(pose.x))
                    if pose.x < 15. or pose.x > 25.:
                        self.doVisualServo(pose.x)
                    else:
                        if self.robot_state == RobotState.VisualServo:
                            if self._carry_item:
                                # Drop item
                                self.setRobotState(RobotState.DropPink)
                            else:
                                rospy.loginfo("Finished visual servoing.")
                                self._stopDriving()
                                if self._current_color == "pink":
                                    # Do reconfiguration
                                    self.setRobotState(RobotState.ReconfigurationT2P)
                                    rospy.loginfo("Running pre-reconfiguration behavior.")
                                    self.setBehavior("Tank", "TankReconf", True)
                                    rospy.loginfo("Send reconfiguration signal.")
                                    self.sendReconfSignal("T2P")
                                elif self._current_color == "green":
                                    self.setRobotState(RobotState.FetchPink)
                        elif self.robot_state == RobotState.VisualServoForRepickup:
                            self.setBehavior("Tank", "TankPickup", True)
                            self._carry_item = True
                            # Rotate the robot after pickup
                            self.setBehavior("Tank", "Tank_diff.xml", False)
                            for i in xrange(15):
                                # Turn left
                                data = Twist()
                                data.angular.z = 1.0
                                self.cmd_vel_pub.publish(data)
                                rospy.sleep(1)
                            for i in xrange(10):
                                # Turn left
                                data = Twist()
                                data.angular.z = 0.0
                                self.cmd_vel_pub.publish(data)
                                rospy.sleep(0.1)
                            self.setBehavior("", "", True)
                            self.setRobotState(RobotState.FindBlue)

            if self.robot_state == RobotState.ReconfigurationT2P:
                if self.isReconfFinished():
                    # Finished reconfiguration
                    # Do pickup action
                    rospy.loginfo("Reconfiguration finished.")
                    self.setBehavior("newPro", "", False)
                    self.setRobotState(RobotState.FetchPink)

            if self.robot_state == RobotState.DropPink:
                self.setBehavior("Tank", "TankDrop", True)
                self._carry_item = False

                # Rotate the robot after drop
                self.setBehavior("Tank", "Tank_diff.xml", False)
                for i in xrange(10):
                    # Turn left
                    data = Twist()
                    data.angular.z = 1.0
                    self.cmd_vel_pub.publish(data)
                    rospy.sleep(1)
                for i in xrange(10):
                    # Turn left
                    data = Twist()
                    data.angular.z = 0.0
                    self.cmd_vel_pub.publish(data)
                    rospy.sleep(0.1)
                self.setBehavior("", "", True)

                self.setRobotState(RobotState.Idle)

            if self.robot_state == RobotState.Done:
                rospy.logerr("What?! You are done!!? How come!!!? You shouldn't be here!")

            if self.robot_state == RobotState.FetchPink:
                if self._current_color == "pink":
                    rospy.sleep(3)
                    self.setBehavior("newPro", "ProTunnelStandup", True)

                    for i in xrange(12):
                        try:
                            pose = rospy.wait_for_message(self.param_dict["pink_obj_topic_name"],
                                                          Vector3, timeout=1.0)
                        except rospy.ROSException:
                            pose = None
                            rospy.logwarn("Cannot find pink object when visual servoing.")

                        if pose is not None:
                            rospy.logdebug("Current x value is {}.".format(pose.x))
                            if pose.x < 15.:
                                self.setBehavior("newPro", "ProTunnelLeft", True)
                            elif pose.x > 25.:
                                self.setBehavior("newPro", "ProTunnelRight", True)
                            else:
                                self.setBehavior("newPro", "ProTunnelForward", True)
                        else:
                            self.setBehavior("newPro", "ProTunnelForward", True)

                        self.setBehavior("newPro", "allMagnets", True)

                    rospy.loginfo("Pickup")
                    self.setBehavior("newPro", "ProTunnelPickup", True)
                    self.setBehavior("newPro", "ProDrop", True)

                    self.setRobotState(RobotState.ReconfigurationP2T)
                    rospy.loginfo("Running pre-reconfiguration behavior.")
                    self.setBehavior("newPro", "ProFlat", True)
                    rospy.loginfo("Send reconfiguration signal.")
                    self.sendReconfSignal("P2T")

                elif self._current_color == "green":
                    self.setBehavior("Tank", "", False)
                    self.setRobotState(RobotState.VisualServoForRepickup)
                else:
                    rospy.logerr("Unrecognized color {}".format(self._current_color))

            if self.robot_state == RobotState.ReconfigurationP2T:
                if self.isReconfFinished():
                    # Finished reconfiguration
                    # Find blue drop zone
                    rospy.loginfo("Reconfiguration finished.")
                    self.setBehavior("Tank", "", False)
                    self.setBehavior("Tank", "TankStandup", True)
                    self.setRobotState(RobotState.VisualServoForRepickup)

            if self.robot_state == RobotState.FindBlue:
                if self.isColorObjDetected("blue"):
                    cached = False
                else:
                    cached = True
                blue_pose = self.getColorObjPose("blue",cached)
                if blue_pose is None:
                    # Explore for blue
                    rospy.loginfo("Never seen blue. Explore.")
                    nbv_pose = self.getNBVPose()
                    self.sendNavGoalRequest(nbv_pose)
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                    self.setRobotState(RobotState.Explore)
                else:
                    # Drive to blue
                    rospy.loginfo("Seen blue before. Drive to it.")
                    self.setRobotState(RobotState.DriveToBlue)
                    rospy.loginfo("Getting blue docking pose.")
                    dock_pose, self.reconf_type = self.getDockPose(blue_pose)
                    rospy.loginfo("Driving to blue.")
                    self.sendNavgoalrequest(dock_pose)
                    self.setbehavior("tank", "tank_diff.xml", false)

            if (self.robot_state == RobotState.DriveToBlue):
                # Driving to the blue object
                state = self.nav_action_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    # Arrived at blue
                    rospy.loginfo("Arrived at blue.")
                    self.setBehavior("", "", False)
                    self.setRobotState(RobotState.VisualServo)
                elif state == GoalStatus.ACTIVE:
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                else:
                    rospy.loginfo("Getting actionlib state {} when going to blue dock.".format(GoalStatus.to_string(state)))

            if (self.robot_state == RobotState.Explore):
                if self.isColorObjDetected("blue") and self._carry_item:
                    # Do blue dock
                    self.setRobotState(RobotState.DriveToBlue)
                    rospy.loginfo("Getting blue docking pose.")
                    dock_pose, self.reconf_type = self.getDockPose(self.getColorObjPose("blue"))
                    rospy.loginfo("Driving to blue.")
                    self.sendNavGoalRequest(dock_pose)
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                elif (not self._carry_item) and ("pink" not in self._done_color) and self.isColorObjDetected("pink"):
                    self._current_color = "pink"
                    # Find pink
                    self.setRobotState(RobotState.DriveToDock)
                    rospy.loginfo("Getting {} docking pose.".format())
                    dock_pose, self.reconf_type = self.getDockPose(self.getColorObjPose(self._current_color))
                    self.sendNavGoalRequest(dock_pose)
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                elif (not self._carry_item) and ("green" not in self._done_color) and self.isColorObjDetected("green"):
                    self._current_color = "green"
                    # Find green
                    self.setRobotState(RobotState.DriveToDock)
                    rospy.loginfo("Getting {} docking pose.".format())
                    dock_pose, self.reconf_type = self.getDockPose(self.getColorObjPose(self._current_color))
                    self.sendNavGoalRequest(dock_pose)
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                else:
                    # Explore
                    state = self.nav_action_client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        # Viewpoint finished.
                        rospy.loginfo("Arrived at point. Stop.")
                        self.setBehavior("", "", False)
                        self.setRobotState(RobotState.Idle)
                    elif state == GoalStatus.ABORTED:
                        # Exploration aborted.
                        rospy.loginfo("Navigation path aborted. Stop.")
                        self.setBehavior("", "", False)
                        self.setRobotState(RobotState.Idle)
                    elif state == GoalStatus.ACTIVE:
                        pass
                    else:
                        rospy.loginfo("Getting actionlib state {} when exploring.".format(GoalStatus.to_string(state)))

            if self.robot_state == RobotState.Idle:
                # Do explore
                self.setRobotState(RobotState.Explore)
                nbv_pose = self.getNBVPose()
                self.sendNavGoalRequest(nbv_pose)
                self.setBehavior("Tank", "Tank_diff.xml", False)
