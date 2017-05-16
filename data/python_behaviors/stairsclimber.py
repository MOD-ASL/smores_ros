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

    SensorBox --> sc4(18)   sc3(23)   sc2(22)   sc1(4) --> Head
                    ^
                    |
         This module is reversed

"""


class StairsClimber:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "sc1":4,
                               "sc2":22,
                               "sc3":23,
                               "sc4":18,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def climbUpStairs(self, c, para_val_dict = {"vel":30, "stairs_height":40}):
        """
        Climb up stairs

        vel:            forward velocity (default=30)
        stairs_height:  the height of the stairs
        """

        time_period = 8

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)
        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)
        time.sleep(time_period)

        return time_period

    def standUp(self, c, para_val_dict = {"stand_angle":40}):
        """
        Stand up the stairsclimber

        stand_angle:    the tilt angle of modules to stand up (default=40)
        """

        time_period = 4

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(90-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        return time_period

    def forward(self, c, para_val_dict = {"vel":30, "stand_angle":40}):
        """
        Drive the proboscis forward

        vel:            forward velocity (default=30)
        stand_angle:    the tilt angle of modules to stand up (default=40)
        """

        time_period = 2

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

        return time_period

if __name__ == "__main__":
    sc = StairsClimber()
    c = SmoresCluster.SmoresCluster(sc.module_mapping.values())
    time.sleep(sc.standUp(c))
    raw_input("Next?")
    while True:
        time.sleep(sc.climbUpStairs(c))
        raw_input("Again?")

