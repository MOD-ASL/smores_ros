#!/usr/bin/env python
from math import pi
import sys
import os
import time
#Add smores library
sys.path.insert(0,"/home/{}/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
from SmoresModule import SmoresCluster


"""
The configuration looks like:

    ch4  SensorBox ch3  ch2   ch1 --> Head (ch5, ch6) will replace ch1

"""


class Chain:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "ch1":8,
                               "ch2":13,
                               "ch3":7,
                               "ch4":3,
                               "ch5":3,
                               "ch6":3,
                               "head":8,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def setHeadModule(self, name):
        self.module_mapping["head"] = self.module_mapping[name]

    def climbDownStairs(self, c, para_val_dict = {"vel":-30, "stairs_height":60}):
        """
        Climb down stairs

        vel:            backward velocity (default=-30)
        stairs_height:  the height of the stairs (default=60)
        """
        return self.climbUpStairs(c, {"vel":para_val_dict["vel"], "stairs_height":para_val_dict["stairs_height"]})

    def run(self,c):
        time.sleep(self.stand(c))
        for i in xrange(3):
            print i
            time.sleep(self.climbUpStairs(c))
        time.sleep(self.dropItem(c))
        for i in xrange(3):
            print i
            time.sleep(self.climbDownStairs(c))

    def climbUpStairsV(self, c, para_val_dict = {"vel":70, "stairs_height":70}):
        """
        Climb up stairs

        vel:            forward velocity (default=60)
        stairs_height:  the height of the stairs (default=70)
        """
        time_period = 8

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.command_velocity("tilt",50, time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch3"]
            c.mods[module_ID].move.command_velocity("tilt",-40, time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])
        time.sleep(time_period)
        time.sleep(1)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.command_velocity("tilt",-50, time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch3"]
            c.mods[module_ID].move.command_velocity("tilt",40, time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

        return time_period

    def climbUpStairs(self, c, para_val_dict = {"vel":70, "stairs_height":70}):
        """
        Climb up stairs

        vel:            forward velocity (default=60)
        stairs_height:  the height of the stairs (default=70)
        """
        time_period = 10

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])
        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

        return time_period

    def flat(self, c, para_val_dict = {}):
        """
        Flaten the chain

        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ch3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

        return time_period

    def stop(self, c, param_dict = {}):
        c.stop()
        return 0.0

    def stand(self, c, para_val_dict = {"stand_angle":40}):
        """
        Stand up the chain

        stand_angle:    the tilt angle of modules to stand up
        """

        time_period = 4

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(80, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ch3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(80, module_ID, "tilt"), time_period)

        return time_period

    def dropItem(self, c, para_val_dict = {}):
        """
        Drop the carried item
        """
        time_period = 4

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)
            c.mods[module_ID].mag.control("top","off")

        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["head"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(70, module_ID, "tilt"), time_period)

        return time_period

    def forward(self, c, para_val_dict = {"vel":30, "stand_angle":40}):
        """
        Drive the chain forward

        vel:            forward velocity (default=30)
        stand_angle:    the tilt angle of modules to stand up (default=40)
        """

        time_period = 2

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["ch2"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ch4"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

        return time_period

if __name__ == "__main__":
    ch = Chain()
    c = SmoresCluster.SmoresCluster(ch.module_mapping.values())
