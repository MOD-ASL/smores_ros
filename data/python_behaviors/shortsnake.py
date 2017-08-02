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

    ss3   ss2   ss1 --> Head

"""


class ShortSnake:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "ss1":11,
                               "ss2":20,
                               "ss3":8,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

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
        c.stop()
        return 0.0

    def stand(self, c, para_val_dict = {"stand_angle":70}):
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
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(70, module_ID, "tilt"), time_period)

        return time_period

    def forward(self, c, para_val_dict = {"vel":50, "stand_angle":50}):
        """
        Drive the shortsnake forward

        vel:            forward velocity (default=30)
        stand_angle:    the tilt angle of modules to stand up (default=50)
        """

        time_period = 2

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["ss1"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ss2"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

            module_ID = self.module_mapping["ss3"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["vel"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["vel"])

        return time_period

if __name__ == "__main__":
    ss = ShortSnake()
    c = SmoresCluster.SmoresCluster(ss.module_mapping.values())
