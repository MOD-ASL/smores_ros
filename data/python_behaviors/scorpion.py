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
                                 sL
    SensorBox --> s4   s3   s2   s1 --> Head
                                 sR

"""


class Scorpion:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "s1":23,
                               "s2":13,
                               "s3":22,
                               "s4":3,
                               "sR":20,
                               "sL":14,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def stop(self, c, param_dict = {}):
        c.stop()
        return 0.0

    def flat(self, c, para_val_dict = {}):
        """
        Flaten the scorpion

        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["s1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["s2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["s3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["s4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sL"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sR"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period

    def stand(self, c, para_val_dict = {"stand_angle":20, "arm_angle":40}):
        """
        Stand up the scorpion

        stand_angle:    the tilt angle of modules to stand up (default=20)
        arm_angle:    the tilt angle of two arm modules to stand up (default=40)
        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["s1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["s2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["s3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["s4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sL"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sR"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period

    def driveWithVW(self, c, v, w, arm_angle=40, stand_angle=20, tilt=False):
        vel_l = v/0.2*50.0-w/0.4*20.0
        vel_r = -v/0.2*50.0-w/0.4*20.0

        return self.drive(c, {"left_V":vel_l, "right_V":vel_r, "arm_angle":arm_angle, "stand_angle":stand_angle, "tilt":tilt})

    def drive(self, c, para_val_dict = {"left_V":30, "right_V":30, "arm_angle":40, "stand_angle":20, "tilt":False}):
        """
        Drive the scorpion

        left_V:       left side velocity (default=30)
        right_V:      right side velocity (default=30)
        arm_angle:    the tilt angle of two arm modules to stand up (default=40)
        """
        time_period = 2

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sL"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["left_V"]*3)

            module_ID = self.module_mapping["sR"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["right_V"]*3)

            module_ID = self.module_mapping["s3"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["left_V"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", para_val_dict["right_V"])

            time.sleep(0.05)

        #if para_val_dict["tilt"]:
        #    for i in xrange(self._cmd_repeat_time):
        #        module_ID = self.module_mapping["s1"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["s2"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["s3"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["s4"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["sL"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["sR"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)
        #        time.sleep(0.05)


        return time_period

if __name__ == "__main__":
    s = Scorpion()
    c = SmoresCluster.SmoresCluster(s.module_mapping.values())
