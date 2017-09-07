import sys, os
sys.path.insert(0,"/home/{}/Projects/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
sys.path.insert(0,"/home/{}/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
from SmoresModule import SmoresCluster
import time
from numpy import pi


mods = [15, 14]

c = SmoresCluster.SmoresCluster(mods)
m = c.mods


while True:
    time.sleep(5)
    print "set"
    m[14].move.command_position("tilt", -40.0/180*pi, 2)
    m[15].move.command_position("tilt", -40.0/180*pi, 2)


