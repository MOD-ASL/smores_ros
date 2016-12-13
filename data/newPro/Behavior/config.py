# config.py
#import sys
#sys.path.append('../..')
from name_map import *

"""
Simulator names:

5
4-0-1-2-3
6
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':middle, 'SMORES_1':front, 'SMORES_2':front_l, 'SMORES_3':front_r,
             'SMORES_4':back, 'SMORES_5':back_l,'SMORES_6':back_r,}
NeutralPositions = {}

disabledDof=[]
speed_scale=0.7
behaviorFiles = {
                 'newProPostReconf.xml':'newProPostReconf.xml',
                 'newProPostReconf2.xml':'newProPostReconf2.xml',
                 'newProTunnel.xml':'newProTunnel.xml',
                 'newProTunnel2.xml':'newProTunnel2.xml',
                }
