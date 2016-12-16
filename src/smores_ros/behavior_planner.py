#!/usr/bin/env python
from math import pi
import time
from aenum import Enum
import sys, os
sys.path.insert(0,"/home/{}/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))

import rospy
from geometry_msgs.msg import Twist
from smores_ros.data_file_loader import DataFileLoader
from smores_library.SmoresModule import SmoresCluster
from smores_library import MissionPlayer
from smores_library.name_map import *
from smores_ros.srv import set_behavior


class RobotBehaviorType(Enum):
    Idle = 0
    Drive = 1
    Action = 2

class BehaviorPlanner(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.robot_configuration_name = ""
        self.robot_behavior_type = None
        self.robot_behavior_name = ""
        self.set_robot_conf_sub = None
        self.MP = None
        self._current_cmd = None
        self._current_data = None

        self._initialize()
        self.main()

    def _initialize(self):
        self.param_name_list = ["~set_behavior_service_name",
                                "~data_file_directory",
                                "~drive_command_topic_name",
                                ]
        self.robot_behavior_type = RobotBehaviorType.Idle

        self._getROSParam()

        self.DFL = DataFileLoader(data_file_directory =
                self.param_dict["data_file_directory"])
        self.DFL.loadAllData()

        # Setup service and subscriber
        rospy.Service(self.param_dict["set_behavior_service_name"],
                set_behavior, self.handle_set_behavior)
        rospy.Subscriber(self.param_dict["drive_command_topic_name"],
                Twist, self.getDriveCMD_cb, queue_size=1)

    def handle_set_behavior(self, request):
        if request.configuration_name == "":
            # This is a reset request
            self.robot_behavior_type = RobotBehaviorType.Idle
            rospy.loginfo("Current robot behavior type {}".format(self.robot_behavior_type))
        elif request.behavior_name == "":
            # This is a change conf request
            self.robot_configuration_name = request.configuration_name
            self.robot_behavior_name = ""
            self.robot_behavior_type = RobotBehaviorType.Idle
            rospy.loginfo("Current robot behavior type {}".format(self.robot_behavior_type))
            self._current_data = self.DFL.data_dict[
                    self.robot_configuration_name]
            self.MP = MissionPlayer.MissionPlayer(
                    self.param_dict["data_file_directory"] +
                    "{}/Behavior".format(self.robot_configuration_name))
        else:
            self.robot_configuration_name = request.configuration_name
            self.robot_behavior_name = request.behavior_name
            if self.MP is None:
                self.MP = MissionPlayer.MissionPlayer(
                        self.param_dict["data_file_directory"] +
                        "{}/Behavior".format(self.robot_configuration_name))
            self._current_data = self.DFL.data_dict[
                    self.robot_configuration_name]
            if request.is_action:
                # This is an action
                self.robot_behavior_type = RobotBehaviorType.Action
                rospy.loginfo("Current robot behavior type {}".format(self.robot_behavior_type))
                rospy.loginfo("Running behavior {}".format(request.behavior_name))
                if request.behavior_name == "TankReconf":
                    self.TankReconf()
                if request.behavior_name == "ProFlat":
                    self._ProFlat()
                if request.behavior_name == "ProTunnelPickup":
                    self.ProTunnelPickup()
                if request.behavior_name == "ProTunnelStandup":
                    self._ProTunnelStandup()
                if request.behavior_name == "allMagnets":
                    self.MP.allMagnets()
                if request.behavior_name == "ProDrop":
                    self.ProDrop()
                if request.behavior_name == "TankDrop":
                    self.TankDrop()
                if request.behavior_name == "TankPickup":
                    self.TankPickup()
                if request.behavior_name == "TankStandup":
                    self._TankPostReconfStandup()
                if request.behavior_name == "ProTunnelLeft":
                    self._ProTunnelForwardWithTurn(direction = "l")
                if request.behavior_name == "ProTunnelRight":
                    self._ProTunnelForwardWithTurn(direction = "r")
                if request.behavior_name == "ProTunnelForward":
                    self._ProTunnelForward()

                rospy.loginfo("Finished behavior {}".format(request.behavior_name))
            else:
                # This is a drive motion
                self.robot_behavior_type = RobotBehaviorType.Drive
                rospy.loginfo("Current robot behavior type {}".format(self.robot_behavior_type))

        return True

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name.lstrip("~")] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))

    def getDriveCMD_cb(self, data):
        if data is not None:
            self._current_cmd = data

    def clip(self, min_v, max_v, data):
        if data == 0.0:
            return data
        if abs(data) > max_v:
            return max_v * np.sign(data)
        if abs(data) < min_v:
            return min_v * np.sign(data)
        return data

    def input2Output(self, para_dict):
        output_mapping = {}
        for input_name, input_value in para_dict["input"].iteritems():
            exec_str = "{}={}".format(input_name, input_value)
            exec(exec_str)
        for output_name, output_value in para_dict["output"].iteritems():
            exec("{}={}".format(output_name, output_value))
            output_mapping[output_name] = eval(output_name)
        return output_mapping

    def main(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.robot_behavior_type == RobotBehaviorType.Idle:
                continue
            if self.robot_behavior_type == RobotBehaviorType.Action:
                continue
            if self.robot_behavior_type == RobotBehaviorType.Drive:
                # Run drive behavior
                if self._current_cmd is None:
                    continue
                para_mapping = self.input2Output(
                        self._current_data.para_dict[
                            self.robot_behavior_name])

                self.MP.playBehavior(self.robot_behavior_name,
                        para_mapping, blocking = False)

    def TankReconf(self):
        self.MP.c.mods[back_r].move.command_position("tilt",5*pi/180,2)
        self.MP.c.mods[back_l].move.command_position("tilt",25*pi/180,2)
        self.MP.c.mods[front].move.command_position("tilt",0*pi/180,2)
        self.MP.c.mods[front_l].move.command_position("tilt",20*pi/180,2)
        self.MP.c.mods[front_r].move.command_position("tilt",0*pi/180,2)
        rospy.sleep(2)

    def ProTunnelPickup(self):
        #self._ProTunnelStandup()
        #rospy.sleep(3)
        #rospy.loginfo("Moving forward")
        #for i in xrange(12):
        #    if i == 11:
        #        rospy.loginfo("Pickup")
        #        self._ProTunnelPickup()
        #    self._ProTunnelForward()
        #    rospy.sleep(2.1)
        #    self.MP.allMagnets("on")
        #    rospy.sleep(0.1)

        i = 0
        for i in xrange(12):
            rospy.loginfo("Moving back")
            self._ProTunnelBack()
            rospy.sleep(2.1)
            self.MP.allMagnets("on")
            rospy.sleep(0.1)

    def ProDrop(self):
        self.MP.c.mods[front_r].move.command_position("tilt", 0.0/180*pi,3)
        rospy.sleep(3)
        rospy.loginfo("Drop")
        self.MP.c.mods[front_r].mag.control("top", "off")
        rospy.sleep(0.1)
        self.MP.c.mods[front_r].move.command_position("tilt", -15.0/180*pi,3)
        rospy.sleep(3.1)
        self.MP.c.mods[front_r].move.command_position("tilt", 0.0/180*pi,3)

        rospy.loginfo("Moving back")
        for i in xrange(2):
            self._ProTunnelBack()
            rospy.sleep(2.1)

    def TankPickup(self):
        rospy.loginfo("Moving forward")
        for i in xrange(10):
            if i == 9:
                rospy.loginfo("Pickup")
                self.MP.c.mods[front].move.command_position("tilt", -5.0/180*pi,3)
            self._TankForward()
            rospy.sleep(2.1)
            self.MP.c.mods[front].mag.control("top", "on")

        i = 0
        for i in xrange(5):
            rospy.loginfo("Moving back")
            self._TankBack()
            rospy.sleep(2.1)

    def TankDrop(self):
        rospy.loginfo("Moving forward")
        for i in xrange(5):
            self._TankForward()
            rospy.sleep(2.1)

        rospy.loginfo("Drop")
        self.MP.c.mods[front].mag.control("top", "off")
        rospy.sleep(0.1)
        self.MP.c.mods[front].move.command_position("tilt", -35.0/180*pi,3)
        rospy.sleep(0.1)
        self.MP.c.mods[front].move.command_position("tilt", -35.0/180*pi,3)
        rospy.sleep(3.1)
        self.MP.c.mods[front].move.command_position("tilt", 0.0/180*pi,3)
        rospy.sleep(0.1)
        self.MP.c.mods[front].move.command_position("tilt", 0.0/180*pi,3)

        rospy.loginfo("Moving back")
        for i in xrange(5):
            self._TankBack()
            rospy.sleep(2.1)

    def _ProTunnelForward(self):
        self.MP.c.mods[back_r].move.command_velocity("pan",-100,2)
        self.MP.c.mods[back_l].move.command_velocity("pan",100,2)
        self.MP.c.mods[front_l].move.command_velocity("left",30,2)
        rospy.sleep(0.01)
        self.MP.c.mods[front_l].move.command_velocity("right",-30,2)
        self.MP.c.mods[front_r].move.command_velocity("left",30,2)
        rospy.sleep(0.01)
        self.MP.c.mods[front_r].move.command_velocity("right",-30,2)

    def _ProTunnelForwardWithTurn(self, direction):
        if direction = "r":
            self.MP.c.mods[back_r].move.command_velocity("pan",-30,2)
        else:
            self.MP.c.mods[back_r].move.command_velocity("pan",-100,2)

        if direction = "l":
            self.MP.c.mods[back_l].move.command_velocity("pan",30,2)
        else:
            self.MP.c.mods[back_l].move.command_velocity("pan",100,2)

        self.MP.c.mods[front_l].move.command_velocity("left",30,2)
        rospy.sleep(0.01)
        self.MP.c.mods[front_l].move.command_velocity("right",-30,2)
        self.MP.c.mods[front_r].move.command_velocity("left",30,2)
        rospy.sleep(0.01)
        self.MP.c.mods[front_r].move.command_velocity("right",-30,2)

    def _ProTunnelBack(self):
        self.MP.c.mods[back_r].move.command_velocity("pan",100,2)
        self.MP.c.mods[back_l].move.command_velocity("pan",-100,2)
        self.MP.c.mods[front_l].move.command_velocity("left",-40,2)
        rospy.sleep(0.01)
        self.MP.c.mods[front_l].move.command_velocity("right",40,2)
        self.MP.c.mods[front_r].move.command_velocity("left",-40,2)
        rospy.sleep(0.01)
        self.MP.c.mods[front_r].move.command_velocity("right",40,2)

    def _ProTunnelPickup(self):
        self.MP.c.mods[front_r].move.command_position("tilt",10*pi/180,2)

    def _ProTunnelStandup(self):
        for i in xrange(3):
            self.MP.c.mods[back_r].move.command_position("tilt",-45*pi/180,2)
            self.MP.c.mods[back_l].move.command_position("tilt",-45*pi/180,2)
            self.MP.c.mods[front].move.command_position("tilt",0*pi/180,2)
            self.MP.c.mods[front_l].move.command_position("tilt",5*pi/180,2)
            self.MP.c.mods[front_r].move.command_position("tilt",-5*pi/180,2)
        rospy.sleep(2)

    def _TankBack(self):
        self.MP.c.mods[back_r].move.command_velocity("pan",90,2)
        self.MP.c.mods[back_l].move.command_velocity("pan",-90,2)
        self.MP.c.mods[front_l].move.command_velocity("pan",-90,2)
        self.MP.c.mods[front_r].move.command_velocity("pan",90,2)

    def _TankForward(self):
        self.MP.c.mods[back_r].move.command_velocity("pan",-90,2)
        self.MP.c.mods[back_l].move.command_velocity("pan",90,2)
        self.MP.c.mods[front_l].move.command_velocity("pan",90,2)
        self.MP.c.mods[front_r].move.command_velocity("pan",-90,2)

    def _TankPostReconfStandup(self):
        self.MP.allMagnets("on")
        rospy.sleep(0.1)

        for i in xrange(3):
            self.MP.c.mods[back_r].move.command_position("tilt",-45*pi/180,2)
            self.MP.c.mods[back_l].move.command_position("tilt",-45*pi/180,2)
        rospy.sleep(2)

        for j in xrange(2):
            for i in xrange(3):
                self.MP.c.mods[back_l].move.command_velocity("pan",40,1)
                self.MP.c.mods[back_r].move.command_velocity("pan",-40,1)
            rospy.sleep(1)
            self.MP.allMagnets("on")
            rospy.sleep(0.1)
            for i in xrange(3):
                self.MP.c.mods[back_l].move.command_velocity("pan",-40,1)
                self.MP.c.mods[back_r].move.command_velocity("pan",40,1)
            rospy.sleep(1)
            self.MP.allMagnets("on")
            rospy.sleep(0.1)

        for i in xrange(3):
            self.MP.c.mods[front_r].move.command_position("tilt",-45*pi/180,2)
            self.MP.c.mods[front_l].move.command_position("tilt",-45*pi/180,2)
        rospy.sleep(2)

    def _ProFlat(self):
        rospy.sleep(2)
        self.MP.c.mods[back_r].move.command_position("tilt",5*pi/180,2)
        self.MP.c.mods[back_l].move.command_position("tilt",25*pi/180,2)
        self.MP.c.mods[front].move.command_position("tilt",0*pi/180,2)
        self.MP.c.mods[front_l].move.command_position("tilt",-5*pi/180,2)
        self.MP.c.mods[front_r].move.command_position("tilt",0*pi/180,2)
        rospy.sleep(2)
