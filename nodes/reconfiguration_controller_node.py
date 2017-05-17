#!/usr/bin/env python
import rospy
from smores_ros import reconfiguration_controller

class SMORESReconfigurationControllerNode:
    def __init__(self):
        self.smores_reconfiguration_controller = None
        self._initialize()

    def _initialize(self):
        # Start the ros node
        rospy.init_node('Reconfiguration_Controller',
                anonymous=True, log_level=rospy.DEBUG)

        self.reconfiguration_controller = reconfiguration_controller.SMORESReconfigurationController()

    def main(self):
        self.reconfiguration_controller.main()

if __name__ == "__main__":
    SRCN = SMORESReconfigurationControllerNode()
    SRCN.main()
