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

    back_l(4)

    back_m(D)   prob_4(D)   prob_3(23)   prob_2(7)   prob_1(14)

    back_r(22)
"""


class Proboscis:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        self.module_mapping = {
                               "back_r":22,
                               "back_l":4,
                               "back_m":1,
                               "prob_1":14,
                               "prob_2":7,
                               "prob_3":23,
                               "prob_4":1,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def climbUpLedge(self, c, para_val_dict = {"vel_back":80, "vel_prob":30, "angle_1":80, "stand_angle":-25}):
        """
        Climb up a ledge

        vel_back:   forward velocity for the two back modules (default=80)
        vel_prob:   forward velocity for all front modules (default=30)
        stand_angle:the tilt angle of two back modules (default=-25)
        angle_up:   the angle value for bending up
        """

        time_period = 8

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.command_velocity("pan", -para_val_dict["vel_back"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.command_velocity("pan", para_val_dict["vel_back"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_2"]
            c.mods[module_ID].move.command_velocity("left", 30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["angle_1"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_3"]
            c.mods[module_ID].move.command_velocity("left", 30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -30, time_period)
        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.command_velocity("pan", -para_val_dict["vel_back"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.command_velocity("pan", para_val_dict["vel_back"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_3"]
            c.mods[module_ID].move.command_velocity("left", 30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["angle_1"], module_ID, "tilt"), time_period)
        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["prob_2"]
            c.mods[module_ID].move.command_velocity("left", 30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -30, time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
        time.sleep(time_period)

    def standUp(self, c, para_val_dict = {"stand_angle":-25}):
        """
        Stand up the proboscis

        stand_angle:   the tilt angle of two back modules (default=-25)
        """

        time_period = 8

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

    def forward(self, c, para_val_dict = {"vel_back":100, "vel_prob":30, "stand_angle":-25}):
        """
        Drive the proboscis forward

        vel_back:   forward velocity for the two back modules (default=100)
        vel_prob:   forward velocity for all front modules (default=30)
        stand_angle:   the tilt angle of two back modules (default=-25)
        """

        time_period = 2

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["back_r"]
            c.mods[module_ID].move.command_velocity("pan", para_val_dict["vel_back"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["back_l"]
            c.mods[module_ID].move.command_velocity("pan", para_val_dict["vel_back"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-25, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["prob_1"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel_prob"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel_prob"], time_period)

            module_ID = self.module_mapping["prob_2"]
            c.mods[module_ID].move.command_velocity("left", para_val_dict["vel_prob"], time_period)
            time.sleep(0.01)
            c.mods[module_ID].move.command_velocity("right", -para_val_dict["vel_prob"], time_period)


if __name__ == "__main__":
    proboscis = Proboscis()
    c = SmoresCluster.SmoresCluster(proboscis.module_mapping.values())
    proboscis.standUp(c)
    raw_input("Next?")
    proboscis.climbUpLedge(c)
