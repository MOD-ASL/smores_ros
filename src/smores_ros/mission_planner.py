#!/usr/bin/env python
from aenum import Enum
import rospy
import actionlib
import tf
import time
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
    DriveToPinkDock = 2
    VisualServo = 3
    Reconfiguration = 4
    FetchPink = 5

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
            pink_obj = rospy.wait_for_message(self.param_dict[color + "_obj_topic_name"],
                                              Vector3, timeout=1.0)
            pose = self.getColorObjPose(color)
            if pose is None:
                return False
            return True
        except rospy.ROSException:
            return False

    def getColorObjPose(self, color):
        try:
            pose, rot = self.tf.lookupTransform("map", color + "Obj", rospy.Time(0))
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
        if x < 20.:
            # Turn left
            data.angular.z = 0.3
            self.cmd_vel_pub.publish(data)
        elif x > 40.:
            # Turn right
            data.angular.z = -0.3
            self.cmd_vel_pub.publish(data)

    def main(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            rate.sleep()
            if self.robot_state == RobotState.DriveToPinkDock:
                # Driving to the pink object
                state = self.nav_action_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    # Arrived at docking point
                    # Do visual servo
                    rospy.loginfo("Finished docking.")
                    self.setBehavior("", "", False)
                    self.setRobotState(RobotState.VisualServo)
                elif state == GoalStatus.ACTIVE:
                    self.setBehavior("Tank", "Tank_diff.xml", False)
                else:
                    rospy.loginfo("Getting actionlib state {} when going to dock.".format(GoalStatus.to_string(state)))

            if self.robot_state == RobotState.VisualServo:
                self.setBehavior("Tank", "Tank_diff.xml", False)
                try:
                    pose = rospy.wait_for_message(self.param_dict["pink_obj_topic_name"],
                                                  Vector3, timeout=1.0)
                except rospy.ROSException:
                    pose = None
                    rospy.logwarn("Cannot find pink object when visual servoing. Stop.")

                if pose is not None:
                    rospy.logdebug("Current x value is {}.".format(pose.x))
                    if pose.x < 20. or pose.x > 40.:
                        self.doVisualServo(pose.x)
                    else:
                        # Do reconfiguration
                        rospy.loginfo("Finished visual servoing.")
                        data = Twist()
                        data.angular.z = 0.0
                        self.cmd_vel_pub.publish(data)
                        self.cmd_vel_pub.publish(data)
                        self.cmd_vel_pub.publish(data)
                        self.setBehavior("", "", False)
                        self.setRobotState(RobotState.Reconfiguration)
                        rospy.loginfo("Running pre-reconfiguration behavior.")
                        self.setBehavior("Tank", "Tank_Reconf.xml", True)
                        rospy.loginfo("Send reconfiguration signal.")
                        self.sendReconfSignal("T2P")

            if self.robot_state == RobotState.Reconfiguration:
                if self.isReconfFinished():
                    # Finished reconfiguration
                    # Do pickup action
                    rospy.loginfo("Reconfiguration finished.")
                    self.setBehavior("newPro", "", False)
                    self.setBehavior("newPro", "newProTunnel.xml", True)

            if (self.robot_state == RobotState.Explore):
                if self.isColorObjDetected("pink"):
                    rospy.loginfo("Pink object detected.")
                    # Do dock
                    self.setRobotState(RobotState.DriveToPinkDock)
                    rospy.loginfo("Getting docking pose.")
                    dock_pose, recon_type = self.getDockPose(self.getColorObjPose("pink"))
                    self.sendNavGoalRequest(dock_pose)
                else:
                    state = self.nav_action_client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        # Viewpoint finished.
                        rospy.loginfo("Arrived at the viewpoint. Stop.")
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
                if self.isColorObjDetected("pink"):
                    rospy.loginfo("Pink object detected.")
                    # Do dock
                    self.setRobotState(RobotState.DriveToPinkDock)
                    rospy.loginfo("Getting docking pose.")
                    dock_pose, recon_type = self.getDockPose(self.getColorObjPose("pink"))
                    self.sendNavGoalRequest(dock_pose)
                else:
                    # Do explore
                    self.setRobotState(RobotState.Explore)
                    nbv_pose = self.getNBVPose()
                    self.sendNavGoalRequest(nbv_pose)
                    self.setBehavior("Tank", "Tank_diff.xml", False)
