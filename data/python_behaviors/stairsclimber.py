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

    SensorBox --> sc4   sc3   sc2   sc1 --> Head

"""


class StairsClimber:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "sc1":23,
                               "sc2":7,
                               "sc3":22,
                               "sc4":1,
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

    def climbUpStairs(self, c, para_val_dict = {"vel":70, "stairs_height":70}):
        """
        Climb up stairs

        vel:            forward velocity (default=60)
        stairs_height:  the height of the stairs (default=70)
        """

        time_period = 10

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)
        time.sleep(time_period)

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stairs_height"], module_ID, "tilt"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

        return time_period

    def flat(self, c, para_val_dict = {}):
        """
        Flaten the stairsclimber

        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
            time.sleep(0.05)
            # Let's also turn the two side wheels to zero position
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "left"), time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "right"), time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period

    def standUp(self, c, para_val_dict = {"stand_angle":40}):
        """
        Stand up the stairsclimber

        stand_angle:    the tilt angle of modules to stand up (default=40)
        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(80, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["sc4"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(80, module_ID, "tilt"), time_period)

        return time_period

    def dropItem(self, c, para_val_dict = {}):
        """
        Drop the carried item
        """
        time_period = 1

        for i in xrange(self._cmd_repeat_time):
            time.sleep(0.05)
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].mag.control("top","off")

        return time_period

    def forward(self, c, para_val_dict = {"vel":30, "stand_angle":40}):
        """
        Drive the stairsclimber forward

        vel:            forward velocity (default=30)
        stand_angle:    the tilt angle of modules to stand up (default=40)
        """

        time_period = 2

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["sc1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

            module_ID = self.module_mapping["sc3"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel"], time_period)
            time.sleep(0.05)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel"], time_period)

        return time_period

if __name__ == "__main__":
    sc = StairsClimber()
    c = SmoresCluster.SmoresCluster(sc.module_mapping.values())
