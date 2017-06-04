#!/usr/bin/env python

import evdev
from evdev import ecodes

import sys
sys.path.append("/home/jim/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python")
from SmoresModule.SmoresCluster import SmoresCluster
import time

c = SmoresCluster(range(1,27))
d = None
repeat = 3


devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
for device in devices:
    if device.name.startswith("Jess Tech"):
        d = evdev.InputDevice(device.fn)

if d is not None:
    for event in device.read_loop():
        if event.value == 01:
            if event.code in [288, 289, 290, 291]:
                print "STOP!"
                for i in xrange(repeat):
                    c.stop()
                    time.sleep(0.05)

            elif event.code in [292, 293, 294, 295]:
                print "Reset!"
                for i in xrange(repeat):
                    c.mux("move.command_position('tilt',0.0,4)")
                    time.sleep(0.05)
                time.sleep(4)
                for i in xrange(repeat):
                    c.stop()
                    time.sleep(0.05)
            else:
                print "Not recognized"

else:
    print "Cannot find device in ", devices
