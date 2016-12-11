#!/usr/bin/env python
import rospy
import time
from smores_ros.srv import nbv_request
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, String

class NBV_Relay(object):
    def __init__(self):
        self.param_dict = {}
        self.param_name_list = []
        self.nbv_result = None

        self._initialize()

    def _getROSParam(self):
        for para_name in self.param_name_list:
            if rospy.has_param(para_name):
                self.param_dict[para_name] = rospy.get_param(para_name)
            else:
                rospy.logerr("Cannot find parameter {}.".format(para_name))
                rospy.signal_shutdown("Cannot find parameter {}.".format(para_name))

    def _initialize(self):
        rospy.init_node("nbv_relay_node", anonymous=True, log_level=rospy.DEBUG)
        self.param_name_list = ["~nbv_request_topic",
                                "~nbv_result_topic",
                                ]
        self._getROSParam()

        # Setup publishers and subscribers
        self.nbv_request_pub = rospy.Publisher(
                                self.param_dict["~nbv_request_topic"],
                                Int32, queue_size=1)
        self.nbv_result_sub = rospy.Subscriber(
                                self.param_dict["~nbv_result_topic"],
                                Pose, self._nbv_request_cb, queue_size=1)

    def _nbv_request_cb(self, pose):
        rospy.loginfo("Received a NBV pose.")
        self.nbv_result = pose

    def handle_nbv_request(self, req):
        rospy.loginfo("Get a NBV request.")

        self.nbv_result = None
        self.nbv_request_pub.publish(0)

        t = time.time()
        while (self.nbv_result is None and (time.time()-t<10.0)):
            time.sleep(0.1)

        if self.nbv_result is None:
            rospy.logerr("NBV Request timeout.")
            rospy.signal_shutdown("NBV Request timeout.")
        else:
            return self.nbv_result

    def nbv_relay_server(self):
        s = rospy.Service("nbv_relay", nbv_request, self.handle_nbv_request)
        rospy.spin()

if __name__ == "__main__":
    nbv_relay = NBV_Relay()
    nbv_relay.nbv_relay_server()
