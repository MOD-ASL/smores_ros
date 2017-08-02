#!/usr/bin/env python

import evdev
from evdev import ecodes

import sys
# Change this to point towards the right location
sys.path.append("/home/jim/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python")
from SmoresModule.SmoresCluster import SmoresCluster
import time
# Create cluster and other required variables for the controller
c = SmoresCluster(range(1,27))
d = None
repeat = 3

# Run until killed
while True:
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    for device in devices:
        # Change this to the right controller name
        #if device.name.startswith("Jess Tech"):
        if device.name.startswith("Microsoft"):
            d = evdev.InputDevice(device.fn)

    if d is not None:
        try:
            for event in device.read_loop():
                if event.value == 01:
                    # When the button is lifted
                    # Change to match the right code
                    #if event.code in [288, 289, 290, 291]:
                    if event.code in [304,305,307,308]:
                    if event.code in [288, 289, 290, 291]:
                        print "STOP!"
                        for i in xrange(repeat):
                            c.stop()
                            time.sleep(0.05)

                    #elif event.code in [292, 293]:
                    elif event.code in [310]:
                        print "Mag!"
                        for i in xrange(repeat):
                            c.allMagnets("on")
                            time.sleep(0.05)

                    #elif event.code in [294, 295]:
                    elif event.code in [311]:
                        print "Reset!"
                        for i in xrange(repeat):
                            c.mux("move.command_position('tilt',0.0,4)")
                            time.sleep(0.05)
                        time.sleep(4)
                        for i in xrange(repeat):
                            c.stop()
                            time.sleep(0.05)
                    else:
                        print "Not recognized {}".format(event.code)
        except:
            print "Replug the controller"

    else:
        print "Cannot find device in ", devices
    time.sleep(1)
