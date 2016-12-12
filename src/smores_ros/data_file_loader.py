#!/usr/bin/env python

import os
import logging
import xml.etree.ElementTree as ET
import rospy
from geometry_msgs.msg import Pose, PoseArray
import yaml

class DataContainer(object):
    def __init__(self):
        self.name = ""
        self.configuration = ""
        self.behavior_dict = {} # file_name: file_path
        self.para_dict = {} # behavior_file_name: parameter_dict
        self.path_dict = {} # file_name: file_path

    def loadConfData(self, path):
        for (dirpath, dirnames, filenames) in os.walk(path):
            for f in filenames:
                if self.configuration != "":
                    rospy.logwarn("Configuration {!r} already exsits.".format(f))
                else:
                    self.configuration = os.path.join(dirpath, f)

    def loadBehaviorData(self, path):
        for (dirpath, dirnames, filenames) in os.walk(path):
            for f in filenames:
                if f == "config.py":
                    continue
                if f.endswith(".yaml"):
                    self.para_dict[f.replace("yaml","xml")] = self._parseBehaviorConfig(os.path.join(dirpath, f))
                elif f in self.behavior_dict.keys():
                    rospy.logwarn("Behavior {!r} already exsits.".format(f))
                else:
                    self.behavior_dict[f] = os.path.join(dirpath, f)

    def loadPathData(self, path):
        for (dirpath, dirnames, filenames) in os.walk(path):
            for f in filenames:
                if f in self.path_dict.keys():
                    rospy.logwarn("Path {!r} already exsits.".format(f))
                else:
                    self.path_dict[f] = self._parsedataDataToPath(os.path.join(dirpath, f))

    def _parsedataDataToPath(self, data):
        tree = ET.parse(data)
        root = tree.getroot() # root is everything in the <data> tag
        xRobotStates = root[0]

        path = []

        for xRobotState in xRobotStates:
            xModuleStates = xRobotState.find('ModuleStates')
            for xModuleState in xModuleStates:
                p = Pose()
                position = xModuleState.find('position')
                p.position.x = float(position.find('x').text)/10
                p.position.y = float(position.find('z').text)/10
                p.position.z = float(position.find('y').text)/10
                rotation = xModuleState.find('rotation')
                p.orientation.x = float(rotation.find('x').text)
                p.orientation.y = float(rotation.find('y').text)
                p.orientation.z = float(rotation.find('z').text)
                p.orientation.w = float(rotation.find('w').text)
                path.append(p)

        return path

    def _parseBehaviorConfig(self, file_path):
        with open(file_path, 'r') as f:
            doc = yaml.load(f)
        return doc

class DataFileLoader(object):
    def __init__(self, data_file_directory=""):

        self.data_file_directory = data_file_directory
        self.data_dict = {}

    def loadData(self, dir_path):
        data = DataContainer()
        rospy.loginfo("Loading data directory: {!r}".format(dir_path))
        for (dirpath, dirnames, filenames) in os.walk(dir_path):
            for dirname in dirnames:
                if dirname == "Configuration":
                    data.loadConfData(os.path.join(dirpath, dirname))
                elif dirname == "Behavior":
                    data.loadBehaviorData(os.path.join(dirpath, dirname))
                elif dirname == "Path":
                    data.loadPathData(os.path.join(dirpath, dirname))
            break

        return data

    def loadAllData(self, data_file_directory=""):
        if data_file_directory == "":
            data_file_directory = self.data_file_directory

        rospy.loginfo("Loading all data in directory: {!r}".format(data_file_directory))
        for (dirpath, dirnames, filenames) in os.walk(data_file_directory):
            for dirname in dirnames:
                if dirname in self.data_dict.keys():
                    rospy.logwarn("Data {!r} already exsits.".format(dirname))
                else:
                    self.data_dict[dirname] = self.loadData(os.path.join(data_file_directory, dirname))
            break
