# config.py
from name_map import *

"""
Simulator names:

5   6
2-0-1 -> front
3   4
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':middle, 'SMORES_1':front, 'SMORES_2':back, 'SMORES_3':back_r,'SMORES_4':front_r, 'SMORES_5':back_l, 'SMORES_6':front_l}
NeutralPositions = {}

disabledDof=['tilt']
speed_scale=0.7
behaviorFiles = {
   'Tank_diff.xml':'Tank_diff.xml',
   'Tank_Reconf.xml':'Tank_Reconf.xml',
   }

