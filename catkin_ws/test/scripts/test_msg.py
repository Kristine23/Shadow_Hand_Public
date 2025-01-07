#!/usr/bin/env python3

import os

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_msgs.msg import MSTAll  # This is the tactile sensor message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker


def tactile_callback(msg):

    data_list = msg.tactiles
    for i in range(len(data_list)):
        data = data_list[i]
        magnitude = data.electrodes
        print('tactile is working', magnitude)


if __name__ == '__main__':
    rospy.init_node('obj_controller',  anonymous=True)

    hand_commander = SrHandCommander("right_hand")

    joints_position = hand_commander.get_joints_position()
    joints_velocity = hand_commander.get_joints_velocity()

    print("Hand joint positions\n" + str(joints_position) + "\n")
    print("Hand joint velocities\n" + str(joints_velocity) + "\n")

    rospy.loginfo('obj_controller has started')

    sub = rospy.Subscriber("/rh/tactile", MSTAll,
                           callback=tactile_callback)

    rospy.spin()
