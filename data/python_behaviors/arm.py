#!/usr/bin/env python
from math import pi
from numpy import sin, sign
import sys
import os
import time
#Add smores library
sys.path.insert(0,"/home/{}/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
sys.path.insert(0,"/home/{}/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
from SmoresModule import SmoresCluster


"""
The configuration looks like:
                  aL
                  a4 SensorBox a3(Reversed)  a2(Reversed)   a1 --> Head
                  aR

"""


class Arm:
    def __init__(self):
        self.module_dof_offset = {
                                 } # module ID_dof_name: offset angle from input cmd
        #self.module_mapping = {
        #                       "a1":10,
        #                       "a2":2,
        #                       "a3":8,
        #                       "a4":18,
        #                       "aR":4,
        #                       "aL":16,
        #                      } # module alias: module ID
        self.module_mapping = {
                               "a1":11,
                               "a2":13,
                               "a3":8,
                               "a4":21,
                               "aR":14,
                               "aL":15,
                              } # module alias: module ID
        self._cmd_repeat_time = 3

    def _get_angle(self, cmd_angle, module_ID, dof_name):
        if str(module_ID) + dof_name in self.module_dof_offset:
            angle_offset = self.module_dof_offset[str(module_ID) + dof_name]
        else:
            angle_offset = 0
        return (cmd_angle + angle_offset) * pi / 180

    def stop(self, c, param_dict = {}):
        for i in xrange(self._cmd_repeat_time):
            c.stop()
            time.sleep(0.05)
        return 0.0

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
                module_ID = self.module_mapping["a1"]
                c.mods[module_ID].move.send_torque("tilt", 15)
        elif para_val_dict["direction"] == "down":
            for i in xrange(self._cmd_repeat_time):
                time.sleep(0.05)
                # Drop the tilt down
                module_ID = self.module_mapping["a1"]
                c.mods[module_ID].move.send_torque("tilt", -15)

        return time_period

    def release(self, c, param_dict = {}):
        """
        Break the ramp connection
        """
        time_period = 5

        # Tilt the front face down
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-60, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        time.sleep(time_period)

        # Turn off first magnet
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].mag.control("top","off")
            time.sleep(0.05)

        return 0.0

    def breakRamp(self, c, param_dict = {}):
        """
        Break the ramp connection
        """
        time_period = 5

        # Turn off first magnet
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].mag.control("top","off")
            time.sleep(0.05)

        # Tilt the front face up
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(70, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        # Tilt the second module face up
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(45, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period

    def breakSensorBox(self, c, param_dict = {}):
        """
        Break the connection with sensor box
        """
        time_period = 1

        # Turn off magnet to sensor module
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].mag.control("bottom","off")
            time.sleep(0.05)

        return time_period

    def pushSensor(self, c, para_val_dict={}):
        """
        Push the sensor box
        """
        time_period = 5

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.send_torque("pan", 90)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.send_torque("pan", -85)

            time.sleep(0.05)

        return time_period

    def openDrawer(self, c, para_val_dict = {}):
        """
        Drive to open the drawer
        """

        time_period = 4

        # Prepare the first module
        module_ID = self.module_mapping["a1"]
        for i in xrange(self._cmd_repeat_time):
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-40, module_ID, "tilt"), time_period)
            time.sleep(0.05)
            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(40, module_ID, "tilt"), time_period)
        time.sleep(time_period)

        # Drive forward for a short time
        self.forward(c, para_val_dict = {"vel":0.25, "stand_angle":50})
        time.sleep(10)

        # Start to tilt up
        module_ID = self.module_mapping["a1"]
        c.mods[module_ID].move.send_torque("tilt", 15)
        # Fire up the magnets
        for a in xrange(7):
            time.sleep(1)
            for i in xrange(self._cmd_repeat_time):
                module_ID = self.module_mapping["a1"]
                c.mods[module_ID].mag.control("top", "on")
            print str(a)
        self.stop(c)
        time.sleep(2)
        self.stop(c)

        # Drive backward
        self.forward(c, para_val_dict = {"vel":-0.1, "stand_angle":50})
        for a in xrange(30):
            time.sleep(1)
            for i in xrange(self._cmd_repeat_time):
                module_ID = self.module_mapping["a1"]
                c.mods[module_ID].mag.control("top", "on")
            print str(a)

        self.forward(c, para_val_dict = {"vel":-0.1, "stand_angle":50})
        for a in xrange(15):
            time.sleep(1)
            for i in xrange(self._cmd_repeat_time):
                module_ID = self.module_mapping["a1"]
                c.mods[module_ID].mag.control("top", "off")
            print str(a)
        self.stop(c)

        return time_period

    def flat(self, c, para_val_dict = {}):
        """
        Flaten the arm

        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a4"]
            #c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period


    def explode(self, c, para_val_dict = {}):

        time_period = 4

        time.sleep(self.flat(c))
        time.sleep(4)

        for i in xrange(self._cmd_repeat_time):
            c.allMagnets("off")
            time.sleep(0.05)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-30, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        time.sleep(time_period)

        for i in xrange(self._cmd_repeat_time):
            c.allMagnets("off")
            time.sleep(0.05)

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        time.sleep(time_period)


        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.send_torque("left", 60)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -30)
            time.sleep(0.05)

            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.send_torque("left", -30)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", 60)
            time.sleep(0.05)

            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.send_torque("left", -60)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", 30)
            time.sleep(0.05)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.send_torque("left", 60)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -30)
            time.sleep(0.05)

            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.send_torque("left", 60)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -30)
            time.sleep(0.05)

        time.sleep(4)

        return time_period

    def stand(self, c, para_val_dict = {"stand_angle":20, "arm_angle":40}):
        """
        Stand up the arm

        stand_angle:    the tilt angle of modules to stand up (default=20)
        arm_angle:    the tilt angle of two arm modules to stand up (default=40)
        """

        time_period = 4

        c.allMagnets("on")
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-10, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(25, module_ID, "tilt"), time_period)
            #c.mods[module_ID].move.command_position("tilt",self._get_angle(10, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a3"]
            #c.mods[module_ID].move.command_position("tilt",self._get_angle(-5, module_ID, "tilt"), time_period)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-10, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a4"]
            #c.mods[module_ID].move.command_position("tilt",self._get_angle(20, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)
            time.sleep(0.05)

        return time_period

    def driveWithVW(self, c, v, w, arm_angle=40, stand_angle=20, tilt=False):
        offset = 0.0
        #
        minimum_w = 0.36
        wiggle_amplitude = 0.02
        if w == 0.0:
            pass
        elif abs(w) <= minimum_w:
            w = minimum_w * sign(w)
            offset =  wiggle_amplitude* sin(2*pi* time.time()) + wiggle_amplitude
            v = v + offset

        vel_l = v/0.2*70.0-w/0.4*25.0
        vel_r = -v/0.2*70.0-w/0.4*25.0

        return self.drive(c, {"left_V":vel_l, "right_V":vel_r, "arm_angle":arm_angle, "stand_angle":stand_angle, "tilt":tilt})

    def pickUp(self, c, para_val_dict = {}):
        ''' Pick up a ramp that is directly in front of the robot'''

        # Set the angle of the head module
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-65, module_ID, "tilt"), 5)
            time.sleep(0.05)
        time.sleep(5)

        c.allMagnets("on")
        time.sleep(0.05)

        # Begin driving forward
        self.drive(c, {"left_V":20, "right_V":-20, "arm_angle":40, "stand_angle":20, "tilt":False})

        # drive for 3 seconds to push the ramp a bit
        time.sleep(3)

        # Perform a slow upward sweep while firing the magnets repeatedly
        # Start to tilt up
        module_ID = self.module_mapping["a1"]
        c.mods[module_ID].move.send_torque("tilt", 15)
        # Fire the magnets four times
        for i in xrange(8):
            time.sleep(1.0)
            for repeats in xrange(self._cmd_repeat_time):
                module_ID = self.module_mapping["a1"]
                c.mods[module_ID].mag.control("top", "on")
            print(str(i))
        # stop
        self.stop(c)

        c.allMagnets("on")
        time.sleep(0.05)

        # tilt up
        self.stand(c)
        time.sleep(2)

        # drive backwards
        time.sleep(self.drive(c, {"left_V":-20, "right_V":20, "arm_angle":40, "stand_angle":20, "tilt":False}))
        #
        return 0.0

    def climbBridge(self, c, para_val_dict = {}):

        time_period = 10
        speed = 40

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.send_torque("left", -10)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", 10)

        # Get everyone in position
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(80, module_ID, "tilt"), time_period)
            # This module is reversed
            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-60, module_ID, "tilt"), time_period)
            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.command_position("tilt",self._get_angle(90, module_ID, "tilt"), time_period)
            time.sleep(0.05)

        time.sleep(time_period)

        # Drive
        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.send_torque("left", speed)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -speed)

            # This module is reversed
            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.send_torque("left", -speed)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", speed)

            # This module is reversed
            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.send_torque("left", -speed)
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", speed)


        return 0.0

    def breakBackThree(self, c, para_val_dict = {}):

        time_period = 4

        for i in xrange(self._cmd_repeat_time):
            c.allMagnets("on")

        time.sleep(self.flat(c, para_val_dict))

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["a4"]
            c.mods[module_ID].mag.control("bottom", "off")
            time.sleep(0.05)

        return 0.0

    def climbRamp(self, c, para_val_dict = {"speed":30}):
        """
        Climb the ramp
        Go forward and bend the second and third module slowly
        """
        time_period = 12

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["speed"]*3)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.send_torque("pan", -para_val_dict["speed"]*3)

            module_ID = self.module_mapping["a1"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["speed"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["speed"])

            # This module is reversed
            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.send_torque("left", -para_val_dict["speed"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", para_val_dict["speed"])
            time.sleep(0.05)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(-30, module_ID, "tilt"), time_period)

            module_ID = self.module_mapping["a3"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["speed"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", -para_val_dict["speed"])
            time.sleep(0.05)
            c.mods[module_ID].move.command_position("tilt",self._get_angle(60, module_ID, "tilt"), time_period)

        return time_period

    def forward(self, c, para_val_dict = {"vel":0.2}):
        """
        Drive the arm forward
        """
        self.driveWithVW(c, para_val_dict["vel"], 0.0)
        return 0.0

    def spinTower(self, c, para_val_dict = {}):
        time_period = 3

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.send_torque("pan", -60)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.send_torque("pan", -60)

        return time_period

    def drive(self, c, para_val_dict = {"left_V":30, "right_V":-30, "arm_angle":40, "stand_angle":20, "tilt":False}):
        """
        Drive the arm

        left_V:       left side velocity
        right_V:      right side velocity
        arm_angle:    the tilt angle of two arm modules to stand up (default=40)
        """
        time_period = 2

        for i in xrange(self._cmd_repeat_time):
            module_ID = self.module_mapping["aL"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["left_V"]*3)

            module_ID = self.module_mapping["aR"]
            c.mods[module_ID].move.send_torque("pan", para_val_dict["right_V"]*3)

            # This module is reversed
            module_ID = self.module_mapping["a2"]
            c.mods[module_ID].move.send_torque("left", para_val_dict["right_V"])
            time.sleep(0.05)
            c.mods[module_ID].move.send_torque("right", para_val_dict["left_V"])
            time.sleep(0.05)

        #if para_val_dict["tilt"]:
        #    for i in xrange(self._cmd_repeat_time):
        #        module_ID = self.module_mapping["a1"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(30, module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["a2"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["a3"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(para_val_dict["stand_angle"], module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["a4"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(0, module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["aL"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)

        #        module_ID = self.module_mapping["aR"]
        #        c.mods[module_ID].move.command_position("tilt",self._get_angle(-para_val_dict["arm_angle"], module_ID, "tilt"), time_period)
        #        time.sleep(0.05)


        return time_period

if __name__ == "__main__":
    a = Arm()
    c = SmoresCluster.SmoresCluster(a.module_mapping.values())
