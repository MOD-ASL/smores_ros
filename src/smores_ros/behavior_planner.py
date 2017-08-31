#!/usr/bin/env python
from math import pi
import time
from aenum import Enum
import sys, os
sys.path.insert(0,"/home/{}/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
sys.path.insert(0,'/home/tarik/Embedded/ecosystem/smores_build/smores_reconfig/python')

import rospy
from geometry_msgs.msg import Twist
try:
    from smores_library.SmoresModule import SmoresCluster
except:
    pass
try:
    from SmoresModule import SmoresCluster
except:
    pass
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
        self._current_cmd = None
        self._current_data = None
        self._cmd_repeat_time = 3
        self.behaivor_dict = {}
        self.c = None

        self._initialize()
        self.main()

    def _initialize(self):
        self.param_name_list = ["~set_behavior_service_name",
                                "~data_file_directory",
                                "~drive_command_topic_name",
                                ]
        self.robot_behavior_type = RobotBehaviorType.Idle

        self._getROSParam()

        sys.path.append(self.param_dict["data_file_directory"])

        from proboscis import Proboscis
        from tank import Tank
        from stairsclimber import StairsClimber
        from scorpion import Scorpion
        from arm import Arm
        from shortsnake import ShortSnake

        self.behaivor_dict["Proboscis"] = Proboscis()
        self.behaivor_dict["Tank"] = Tank()
        self.behaivor_dict["StairsClimber"] = StairsClimber()
        self.behaivor_dict["Scorpion"] = Scorpion()
        self.behaivor_dict["Arm"] = Arm()
        self.behaivor_dict["ShortSnake"] = ShortSnake()

        self.c = SmoresCluster.SmoresCluster(
                self.behaivor_dict["Arm"].module_mapping.values(), batteries=False)

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
        else:
            self.robot_configuration_name = request.configuration_name
            self.robot_behavior_name = request.behavior_name

            if request.is_action:
                # This is an action
                self.robot_behavior_type = RobotBehaviorType.Action
                rospy.loginfo("Current robot behavior type {}".format(self.robot_behavior_type))
                rospy.loginfo("Running behavior {}".format(request.behavior_name))

                b = self.behaivor_dict[request.configuration_name]

                if request.behavior_name == "flat":
                    time.sleep(b.flat(self.c))
                elif request.behavior_name == "stand":
                    time.sleep(b.stand(self.c))
                elif request.behavior_name == "stop":
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "climbUpBox":
                    time.sleep(b.climbUpLedge(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "climbDownBox":
                    time.sleep(b.climbDownLedge(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "drop":
                    time.sleep(b.dropItem(self.c))
                elif request.behavior_name == "climbUpStairs":
                    time.sleep(b.climbUpStairs(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "climbDownStairs":
                    time.sleep(b.climbDownStairs(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "pickUp":
                    time.sleep(b.pickUp(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "dropRamp":
                    time.sleep(b.dropRamp(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "breakRamp":
                    time.sleep(b.breakRamp(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "climbRamp":
                    time.sleep(b.climbRamp(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "bend":
                    time.sleep(b.bend(self.c))
                    # No stop here
                elif request.behavior_name == "spinCWF":
                    # No sleep here
                    b.spin(self.c, {"direction":"cw", "speed":20})
                    # No stop here
                elif request.behavior_name == "spinCCWF":
                    # No sleep here
                    b.spin(self.c, {"direction":"ccw", "speed":20})
                    # No stop here
                elif request.behavior_name == "spinCWS":
                    # No sleep here
                    b.spin(self.c, {"direction":"cw", "speed":15})
                    # No stop here
                elif request.behavior_name == "spinCCWS":
                    # No sleep here
                    b.spin(self.c, {"direction":"ccw", "speed":15})
                    # No stop here
                elif request.behavior_name == "adjustHeadTiltUP":
                    # No sleep here
                    b.adjustHeadTilt(self.c, {"direction":"up"})
                    # No stop here
                elif request.behavior_name == "adjustHeadTiltDOWN":
                    # No sleep here
                    b.adjustHeadTilt(self.c, {"direction":"down"})
                    # No stop here
                elif request.behavior_name == "openDrawer":
                    time.sleep(b.openDrawer(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "preSpin":
                    time.sleep(b.preSpin(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "pushSensor":
                    time.sleep(b.pushSensor(self.c))
                    # No stop here
                elif request.behavior_name == "breakSensorBox":
                    time.sleep(b.breakSensorBox(self.c))
                    time.sleep(b.stop(self.c))
                elif request.behavior_name == "forward":
                    time.sleep(b.forward(self.c))
                    # No stop here
                else:
                    rospy.logwarn("Cannot find behavior {}".format(request.behavior_name))

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

    def main(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.robot_behavior_type == RobotBehaviorType.Drive:
                # Run drive behavior
                if self._current_cmd is None:
                    continue

                b = self.behaivor_dict[self.robot_configuration_name]
                b.driveWithVW(self.c, self._current_cmd.linear.x,
                            self._current_cmd.angular.z)
        if self.robot_configuration_name != "":
            b = self.behaivor_dict[self.robot_configuration_name]
            b.stop(self.c)
