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

    ss3   ss2(Reversed)   ss1 --> Head

"""


class ShortSnake:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "ss1":11,
                               "ss2":13,
                               "ss3":21,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def climbUpStairs(self, c, para_val_dict = {"vel":60, "stairs_height":80}):
        """
        Climb up stairs

        vel:            forward velocity
        stairs_height:  the height of the stairs
        """
        time_period = 10

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["ss1"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

        return time_period

    def flat(self, c, para_val_dict = {}):
        """
        Flaten the shortsnake

        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["ss1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period

    def stop(self, c, param_dict = {}):
        for i in xrange(self._cmd_repeat_time):
            c.stop()
            time.sleep(0.05)
        return 0.0

    def bend(self, c, para_val_dict = {"stand_angle":-70}):
        """
        Bend the middle module more

        stand_angle:    the tilt angle of modules to stand up
        """

        time_period = 4

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        return time_period

    def preSpin(self, c, para_val_dict = {}):
        """
        Prepare the shortsnake before spin in place
        """
        time_period = 6

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            # Bend the middle module up
            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(25, module_ID, "tilt"), time_period)
            time.sleep(0.05)
            # Bend the last module flat
            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            # Bend the front module down and hopefully show the apriltag
            module_ID = self.module_mapping["ss1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

        return time_period

    def spin(self, c, para_val_dict = {"direction":"cw", "speed":20}):
        """
        Spin the shortsnake about the middle module

        direction:  the direction of spinning
        """
        time_period = 2

        if para_val_dict["direction"] == "ccw":
            for i in xrange(self._cmd_repeat_time):
                # This module is reversed
                module_ID = self.module_mapping["ss2"]
                c.mods[module_ID].move.send_torque("left", -para_val_dict["speed"])
                time.sleep(0.05)
                c.mods[module_ID].move.send_torque("right", -para_val_dict["speed"])
        elif para_val_dict["direction"] == "cw":
            for i in xrange(self._cmd_repeat_time):
                # This module is reversed
                module_ID = self.module_mapping["ss2"]
                c.mods[module_ID].move.send_torque("left", para_val_dict["speed"])
                time.sleep(0.05)
                c.mods[module_ID].move.send_torque("right", para_val_dict["speed"])

        return time_period

    def crazy(self, c):
        for i in xrange(self._cmd_repeat_time):
            # This module is reversed
            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.send_torque("left", 100)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -100)
        time.sleep(0.2)
        self.stop(c)

    def adjustHeadTilt(self, c, para_val_dict = {"direction":"up"}):
        """
        Adjust the tilt angle of the first module to flat

        direction:  the direction of the tilt movement
        """
        time_period = 2

        if para_val_dict["direction"] == "up":
            for i in xrange(self._cmd_repeat_time):
                time.sleep(0.05)
                # Lift the tilt up
                module_ID = self.module_mapping["ss1"]
                c.mods[module_ID].move.send_torque("tilt", 15)
        elif para_val_dict["direction"] == "down":
            for i in xrange(self._cmd_repeat_time):
                time.sleep(0.05)
                # Drop the tilt down
                module_ID = self.module_mapping["ss1"]
                c.mods[module_ID].move.send_torque("tilt", -15)

        return time_period


    def openDrawer(self, c, para_val_dict = {}):
        """
        Drive to open the drawer
        """

        time_period = 2

        # Drive forward for a short time
        self.forward(c, para_val_dict = {"vel":60, "stand_angle":50})
        time.sleep(4)

        # Start to tilt up
        module_ID = self.module_mapping["ss1"]
        c.mods[module_ID].move.send_torque("tilt", 15)
        # Fire up the magnets
        for i in xrange(4):
            time.sleep(1.0)
            for i in xrange(self._cmd_repeat_time):
                module_ID = self.module_mapping["ss1"]
                c.mods[module_ID].mag.control("top", "on")
        self.stop(c)

        # Drive backward
        self.forward(c, para_val_dict = {"vel":-60, "stand_angle":50})
        time.sleep(5)
        self.stop(c)

        return time_period


    def stand(self, c, para_val_dict = {"stand_angle":-30}):
        """
        Stand up the shortsnake

        stand_angle:    the tilt angle of modules to stand up
        """

        time_period = 4

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["ss1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(70, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(80, module_ID, "tilt"), time_period)

        return time_period

    def backward(self, c, para_val_dict = {"vel":-50, "stand_angle":50}):
        self.forward(c, para_val_dict)

    def forward(self, c, para_val_dict = {"vel":50, "stand_angle":50}):
        """
        Drive the shortsnake forward

        vel:            forward velocity (default=30)
        stand_angle:    the tilt angle of modules to stand up (default=50)
        """

        time_period = 2

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["ss1"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            # This module is reversed
            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.send_torque("left", -para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", para_val_dict["vel"])

            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

        return time_period

if __name__ == "__main__":
    ss = ShortSnake()
    c = SmoresCluster.SmoresCluster(ss.module_mapping.values())
