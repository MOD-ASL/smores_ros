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

    back_l            front_l

    back_m   mid      front_m

    back_r            front_r
"""


class Tank:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "back_r":1,
                               "back_l":11,
                               "back_m":5,
                               "mid":5,
                               "front_l":22,
                               "front_m":23,
                               "front_r":20,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def flat(self, c, para_val_dict = {}):
        """
        Flaten the tank
        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["front_l"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["front_r"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

        return time_period

    def stand(self, c, para_val_dict = {"stand_angle":-45}):
        """
        Stand up the tank

        stand_angle:   the tilt angle of two back modules
        """

        time_period = 8

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["front_r"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["front_l"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        return time_period

    def driveWithVW(self, c, v, w, stand_angle=-45, tilt=False):
        vel_l = v/0.2*150.0-w/0.4*50.0
        vel_r = -v/0.2*150.0-w/0.4*50.0

        return self.drive(c, {"vel_l":vel_l, "vel_r":vel_r, "stand_angle":stand_angle, "tilt":tilt})

    def stop(self, c, param_dict = {}):
        c.stop()
        return 0.0

    def drive(self, c, para_val_dict = {"vel_l":30, "vel_r":30, "stand_angle":-45, "tilt":False}):
        """
        Drive the tank forward
        """

        time_period = 2

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["vel_r"])
            if para_val_dict["tilt"]:
                time.sleep(0.05)
                c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["vel_l"])
            if para_val_dict["tilt"]:
                time.sleep(0.05)
                c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["front_r"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["vel_r"])
            if para_val_dict["tilt"]:
                time.sleep(0.05)
                c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["front_l"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["vel_l"])
            if para_val_dict["tilt"]:
                time.sleep(0.05)
                c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        return time_period


if __name__ == "__main__":
    tank = Tank()
    c = SmoresCluster.SmoresCluster(tank.module_mapping.values())
