#!/usr/bin/env python3

import os

import rospy
from sr_robot_msgs.msg import MSTAll  # This is the tactile sensor message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker


def tactile_callback(msg):
    global current_force

    data_list = msg.tactiles
    for i in range(len(data_list)):
        data = data_list[i]
        magnitude = data.electrodes
        max_force = 0
        for i in range(0, len(magnitude)):
            force = magnitude[i]
            if magnitude[i] > max_force:
                max_force = magnitude[i]


if __name__ == '__main__':
    rospy.init_node('obj_controller')

    global current_force
    global writeble

    pup = rospy.Publisher("/rh_trajectory_controller/command",
                          JointTrajectory, queue_size=10)

    sub = rospy.Subscriber("/rh/tactile", MSTAll,
                           callback=tactile_callback)

    rospy.loginfo('obj_controller has started')

    rospy.spin()
