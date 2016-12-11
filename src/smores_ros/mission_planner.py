#!/usr/bin/env python
from aenum import Enum
import rospy
import actionlib
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Vector3
from smores_ros.srv import nbv_request, target_req
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotState(Enum):
    Idle = 0
    Explore = 1
    DriveToPinkDock = 2
    Reconfiguration = 3
    FetchPink = 4

class MissionPlanner(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.nav_action_client = None
        self.reconf_signal_pub = None
        self.robot_state = None

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
                                ]
        self.robot_state = RobotState.Idle

        self._getROSParam()

        # Setup client, publishers and subscribers
        self.nav_action_client = actionlib.SimpleActionClient(
                self.param_dict["navigation_action_name"], MoveBaseAction)
        self.reconf_signal_pub = rospy.Publisher(
                self.param_dict["reconf_signal_topic_name"], String)

        # Waiting for all service to be ready
        rospy.loginfo("Waiting for nbv pose service ...")
        rospy.wait_for_service(self.param_dict["nbv_service_name"])
        rospy.loginfo("Waiting for dock point service ...")
        #rospy.wait_for_service(self.param_dict["dock_point_service_name"])
        rospy.loginfo("Waiting for navigation action service ...")
        self.nav_action_client.wait_for_server()

    def setRobotState(self, new_state):
        rospy.loginfo("Switching robot state from {} to {}."
                .format(self.robot_state, new_state))
        self.robot_state = new_state

    def isPinkObjDetected(self):
        try:
            pink_obj = rospy.wait_for_message(self.param_dict["pink_obj_topic_name"],
                                              Vector3, timeout=1.0)
            return True
        except rospy.ROSException:
            return False

    def isBlueObjDetected(self):
        try:
            blue_obj = rospy.wait_for_message(self.param_dict["blue_obj_topic_name"],
                                              Vector3, timeout=1.0)
            return True
        except rospy.ROSException:
            return False

    def getNBVPose(self):
        try:
            getNBV = rospy.ServiceProxy(
                    self.param_dict["nbv_service_name"], nbv_request)
            resp = getNBV(0)
            return resp.nbv_pose
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def getDockPose(self):
        try:
            get_dock = rospy.ServiceProxy(
                    self.param_dict["dock_point_service_name"], target_req)
            resp = get_dock(0)
            return resp.nav_point, resp.recon_type
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

    def main(self):
        while not rospy.is_shutdown():

            rospy.sleep(3)

            if self.isPinkObjDetected():
                rospy.loginfo("Pink object detected.")
                # Do dock
                self.setRobotState(RobotState.DriveToPinkDock)
                rospy.loginfo("Getting docking pose.")
                dock_pose, recon_type = self.getDockPose()
                self.sendNavGoalRequest(dock_pose)
            else:
                # Do explore
                self.setRobotState(RobotState.Explore)
                nbv_pose = self.getNBVPose()
                self.sendNavGoalRequest(nbv_pose)

            while not rospy.is_shutdown():
                print self.nav_action_client.get_state()
                rospy.sleep(1)

