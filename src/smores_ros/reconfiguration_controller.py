import tf
import rospy
import time
import os
import sys
import yaml
from numpy import sqrt
from math import pi
from smores_ros import smores_controller
import rospkg
from std_msgs.msg import String

class SMORESReconfigurationController:
    def __init__(self):
        self.tf = None
        self.smores_controller = None
        self.reconf_waitlist = []
        self._current_waitlist_id = 0
        self.tag_module_mapping = {}
        self.reconf_pub = None
        self.path = None
        self._repeat_cmd_num = 3
        self.do_reconf = False
        self.path_dict = {}
        self.reconf_order_data = {}
        self.reconf_path_data_path = ""
        self._current_reconf_direction=""
        self._module_heading_angle = None

        self._initialize()

    def _initialize(self):
        self.tf = tf.TransformListener()
        self.reconf_order_data = {
                "SC2S":[
                {"move_m":"tag_4",
                 "target_m":"tag_5",
                 "target_connect":"left",
                 "move_connect":"bottom",
                 "move_heading":pi/2,
                },
                {"move_m":"tag_6",
                 "target_m":"tag_5",
                 "target_connect":"right",
                 "move_connect":"bottom",
                 "move_heading":-pi/2,
                },
                ],
                "S2SC":[
                {"move_m":"tag_6",
                 "undock_m":"tag_5",
                 "move_disconnect":"bottom",
                 "undock_disconnect":"right"
                },
                {"move_m":"tag_4",
                 "undock_m":"tag_5",
                 "move_disconnect":"bottom",
                 "undock_disconnect":"left"
                },
                ],
                "T2P":[
                {"move_m":"tag_4",
                 "target_m":"tag_5",
                 "undock_m":"tag_5",
                 "target_connect":"top",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":0.0,
                 "undock_disconnect":"left"
                },
                {"move_m":"tag_6",
                 "target_m":"tag_4",
                 "undock_m":"tag_5",
                 "target_connect":"top",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":0.0,
                 "undock_disconnect":"right"
                }
                ],
                "P2T":[
                {"move_m":"tag_6",
                 "target_m":"tag_5",
                 "undock_m":"tag_4",
                 "target_connect":"right",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":-pi/2,
                 "undock_disconnect":"top"
                },
                {"move_m":"tag_4",
                 "target_m":"tag_5",
                 "undock_m":"tag_5",
                 "target_connect":"left",
                 "move_disconnect":"bottom",
                 "move_connect":"bottom",
                 "move_heading":pi/2,
                 "undock_disconnect":"top"
                }
                ]
                }

        self.up_angle = {23:10*pi/180.0, 20:10*pi/180.0, 14:10*pi/180}
        self.neutral_angle = {23:0.0, 20:0.0, 14:0.0*pi/180}
        self.tag_module_mapping = {"tag_4":14, "tag_5":23, "tag_6":20}
        self.smores_list = self.tag_module_mapping.values()

        rospy.Subscriber('{}/reconf_signal'.format(rospy.get_name()), String, self._reconf_signal_callback)
        #rospy.on_shutdown(self.onShutdown)
        self.reconf_path_data_path = os.path.join(rospkg.RosPack().get_path("smores_ros"), "data", "reconf_path.yaml")
        self.loadReconfPath()

        rospy.sleep(0.3)

    def loadReconfPath(self):
        rospy.loginfo("Loading path file from {}...".format(self.reconf_path_data_path))
        with open(self.reconf_path_data_path, "r") as pathfile:
            self.path_dict = yaml.load(pathfile)

    def _reconf_signal_callback(self, data):
        data = str(data.data).split(":")

        if data[0] in self.reconf_order_data.keys():
            self.reconf_waitlist = self.reconf_order_data[data[0]]
            self._current_reconf_direction = data[0]
            self.do_reconf = True
            self._current_waitlist_id = int(data[1])
        else:
            rospy.logwarn("Cannot recognize {}".format(data))

    def getTagPosition(self, tag_id):
        rospy.logdebug("Getting position for {!r}".format(tag_id))
        while not rospy.is_shutdown():
            try:
                return self.tf.lookupTransform("tag_5", tag_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return None

    def lowPassFilter(self, angle_in, angle_out, ALPHA):
        if angle_out is None:
            return angle_in
        angle_out = angle_out + ALPHA * (angle_in - angle_out)
        return angle_out

    def _driveToTargetPoint(self, move_tag, pt_x, pt_y, timeout = 30.0, near_enough = 0.01):

        arrived = False
        error_code = ""
        start_time = time.time()
        move_module_id = self.tag_module_mapping[move_tag]
        rate = rospy.Rate(2)

        while not arrived:
            rate.sleep()
            if (time.time() - start_time) > timeout:
                rospy.logwarn("Drive to point timed out.")
                error_code = "TIMEOUT"
                break

            if (rospy.is_shutdown()):
                rospy.logwarn("Drive to point interupted.")
                error_code = "INTERUPT"
                break

            rospy.loginfo("Driving to point {}".format([pt_x, pt_y]))
            try:
                (move_pose, move_rot) = self.getTagPosition(move_tag)
            except TypeError as e:
                rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
                self.smores_controller.stopAllMotors(move_module_id)
                continue

            x = pt_x - move_pose[0]
            y = pt_y - move_pose[1]
            theta = tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]
            rospy.loginfo("Pose is {:.4f} and {:.4f} and {:.4f}".format(move_pose[0], move_pose[1], theta))

            [v,w] = self.smores_controller.global2Local(x, y, theta)
            self.smores_controller.driveWithLocal(move_module_id, v, w)

            if ((sqrt(x**2+y**2)<near_enough)):
                rospy.logdebug("Target point arrived.")
                arrived = True

        self.smores_controller.stopAllMotors(move_module_id)
        return arrived, error_code

    def _undock(self, reconf_data):

        if "undock_m" not in reconf_data:
            # This reconfiguration does not need undocking process
            return

        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_disconnect = reconf_data["move_disconnect"]
        undock_module_id = self.tag_module_mapping[reconf_data["undock_m"]]
        undock_module_obj = self.smores_controller.getModuleObjectFromID(undock_module_id)
        undock_module_disconnect = reconf_data["undock_disconnect"]

        # Turn off all motor for magnet control
        self.smores_controller.stopAllMotors(move_module_obj)
        self.smores_controller.stopAllMotors(undock_module_obj)

        # Disconnect undock module magnets
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.mag.control(move_module_disconnect, "off")
            undock_module_obj.mag.control(undock_module_disconnect, "off")
            time.sleep(0.05)

        # Stop for a bit before drive
        time.sleep(0.1)

        # Drop down the front wheel to detach from others
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", -45*pi/180, 3)
            time.sleep(0.05)
        time.sleep(3)

        # Lift up the front wheel for better driving
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", self.up_angle[move_module_id], 2)
            time.sleep(0.05)
        time.sleep(3)
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.send_torque("tilt", 0.0)
            time.sleep(0.05)

        # Spin front wheel
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("pan", 0, 4)
            time.sleep(0.05)
        time.sleep(5)
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.send_torque("pan", 0.0)
            time.sleep(0.05)

        # Drive the move module out to avoid collision
        self.smores_controller.driveForward(move_module_obj)
        time.sleep(2.0)
        self.smores_controller.stopAllMotors(move_module_obj)

    def _preDock(self, reconf_data):
        if "target_m" not in reconf_data:
            # This reconfiguration does not need docking process
            return

        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_connect = reconf_data["move_connect"]
        target_module_id = self.tag_module_mapping[reconf_data["target_m"]]
        target_module_obj = self.smores_controller.getModuleObjectFromID(target_module_id)
        target_module_connect = reconf_data["target_connect"]
        move_heading = reconf_data["move_heading"]

        # Correct the heading
        for i in xrange(self._repeat_cmd_num):
            rospy.loginfo("Check heading ...")
            self.correctHeading(reconf_data["move_m"], move_module_obj, move_heading)
            rospy.sleep(1.)

        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", self.neutral_angle[move_module_id], 3)
            time.sleep(0.05)
        time.sleep(4)

        # Turn the target face to zero
        face = target_module_connect
        if target_module_connect == "top":
            face = "pan"
        if face != "bottom":
            for i in xrange(self._repeat_cmd_num):
                target_module_obj.move.command_position(face, 0.0, 4)
                time.sleep(0.05)
            time.sleep(4)
            for i in xrange(self._repeat_cmd_num):
                move_module_obj.move.send_torque(face, 0.0)
                time.sleep(0.05)

        # Turn off all motor for magnet control
        self.smores_controller.stopAllMotors(move_module_obj)
        self.smores_controller.stopAllMotors(target_module_obj)

        # Turn on magnects for new connections
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.mag.control(move_module_connect, "on")
            target_module_obj.mag.control(target_module_connect, "on")


    def _dock(self, reconf_data):
        if "target_m" not in reconf_data:
            # This reconfiguration does not need docking process
            return

        move_module_id = self.tag_module_mapping[reconf_data["move_m"]]
        move_module_obj = self.smores_controller.getModuleObjectFromID(move_module_id)
        move_module_connect = reconf_data["move_connect"]
        target_module_id = self.tag_module_mapping[reconf_data["target_m"]]
        target_module_obj = self.smores_controller.getModuleObjectFromID(target_module_id)
        target_module_connect = reconf_data["target_connect"]

        # Wiggle the tilt for better docking
        self.smores_controller.driveBackward(move_module_obj)
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", -15*pi/180 + self.neutral_angle[move_module_id], 2)
            time.sleep(0.05)
        time.sleep(2)
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.move.command_position("tilt", self.neutral_angle[move_module_id], 2)
            time.sleep(0.05)
        time.sleep(3)

        # Turn off all motor for magnet control
        self.smores_controller.stopAllMotors(move_module_obj)
        self.smores_controller.stopAllMotors(target_module_obj)

        # Turn on magnects for new connections
        for i in xrange(self._repeat_cmd_num):
            move_module_obj.mag.control(move_module_connect, "on")
            target_module_obj.mag.control(target_module_connect, "on")

    def getTagOrientation(self, move_tag):
        try:
            (move_pose, move_rot) = self.getTagPosition(move_tag)
        except TypeError as e:
            rospy.logerr("Cannot find position for {!r}: {}".format(move_tag, e))
            return None
        return tf.transformations.euler_from_quaternion(move_rot, 'szyx')[0]

    def correctHeading(self, move_tag, move_module_obj, target_angle, close_enough=0.1):
        rospy.loginfo("Correct heading of {}".format(move_tag))
        angle = 2*pi
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and abs(target_angle - angle)>close_enough:
            angle = self.getTagOrientation(move_tag)
            if angle is None:
                continue
            if target_angle - angle < 0.:
                # spin ccw
                self.smores_controller.spin(move_module_obj, direction = "ccw")
            else:
                # spin cw
                self.smores_controller.spin(move_module_obj, direction = "cw")
            rospy.loginfo("diff is {}".format(target_angle-angle))
            rate.sleep()
        rospy.loginfo("Heading is good!")
        self.smores_controller.stopAllMotors(move_module_obj)

    def main(self):
        rate = rospy.Rate(2)
        while not (rospy.is_shutdown() or self.do_reconf):
            rate.sleep()
            rospy.loginfo("Waiting for reconfiguration signal on topic {}/reconf_signal".format(rospy.get_name()))

        if not self.do_reconf:
            return

        if self.smores_controller is None:
            self.smores_controller = smores_controller.SMORESController(self.smores_list)

        reconf_data = self.reconf_waitlist[self._current_waitlist_id]

        rospy.logdebug("Performing undock ...")
        self._undock(self.reconf_waitlist[self._current_waitlist_id])

        rospy.logdebug("Loading reconf path ...")
        self.path = self.path_dict[self._current_reconf_direction + "_" + reconf_data["move_m"]]

        rospy.logdebug("Start to follow a path ...")
        for pt_id, pt in enumerate(self.path):
            x = pt[0]/100.0
            y = pt[1]/100.0

            move_tag = self.reconf_waitlist[self._current_waitlist_id]["move_m"]

            if pt_id  == (len(self.path) - 1):
                self._preDock(self.reconf_waitlist[self._current_waitlist_id])
                self._driveToTargetPoint(move_tag, x, y, timeout = 10.0, near_enough = 0.002)
            else:
                arrived, error_code = self._driveToTargetPoint(move_tag, x, y)

            if pt_id  == (len(self.path) - 2) and error_code != "INTERUPT":
                # Do it again
                rospy.sleep(3)
                rospy.loginfo("Go to the point again, just in case.")
                self._driveToTargetPoint(move_tag, x, y)

            #raw_input("Arrived at point {},{}".format(x,y))

        # This is for quicker ctrl-C
        if rospy.is_shutdown():
            return

        rospy.logdebug("Finishing docking ...")
        self._dock(self.reconf_waitlist[self._current_waitlist_id])
        rospy.logdebug("Waiting for octomap to stablize ...")
        time.sleep(10)
        self.path = None

        #self.onShutdown()

        rospy.logdebug("Reconfiguration Finished!")
        pub = rospy.Publisher("{}/reconf_status".format(rospy.get_name()), String, queue_size=10)
        for i in xrange(100):
            pub.publish(String("{} is done.".format(self._current_waitlist_id)))
            rospy.sleep(0.1)


    def onShutdown(self):
        rospy.loginfo("Shutting down {}".format(rospy.get_name()))

        # Stop all modules
        if self.smores_controller is None:
            return
        for i in self.smores_list:
            self.smores_controller.stopAllMotors(self.smores_controller.getModuleObjectFromID(i))

