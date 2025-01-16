#!/usr/bin/env python3

import os
from threading import Lock, Thread

import Main
import numpy as np
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_msgs.msg import MSTAll  # This is the tactile sensor message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

mutex1 = Lock()
mutex2 = Lock()


class CallBack_Functions():

    robot_force = np.zeros(5)
    robot_pos = [np.zeros(3) for i in range(5)]

    def tactile_callback(self, msg):
        data_list = msg.tactiles
        forces = [0 for i in range(5)]
        for i in range(len(data_list)):
            data = data_list[i]
            magnitude += 0
            for i in range(len(data.magnetic_data)):
                vec = np.array(
                    [data.magnetic_data[i].x, data.magnetic_data[i].y, data.magnetic_data[i].z])
                magnitude += np.linalg.norm(vec)
            magnitude = magnitude/len(data.magnetic_data)
            force = 803.072182237894*magnitude - 0.27123231139724746
            with mutex1:
                forces[i] = force
            print(magnitude)
            print('tactile is working')

    def pose_callback(self, msg):
        pose = msg.pose
        xyz = pose.position
        position = np.array([xyz.x, xyz.y, xyz.z])
        print(position)


if __name__ == '__main__':
    rospy.init_node('obj_controller')

    # hand_commander = SrHandCommander("right_hand")

    # joints_position = hand_commander.get_joints_position()
    # joints_velocity = hand_commander.get_joints_velocity()

    # print(joints_position['rh_FFJ1'])

    call_back = CallBack_Functions()

    sub = rospy.Subscriber("/rh/tactile", MSTAll,
                           callback=call_back.tactile_callback)

    sub = rospy.Subscriber("/sdu/tactile/ff_cop", Marker,
                           callback=call_back.pose_callback)

    pup = rospy.Publisher("/rh_trajectory_controller/command",
                          JointTrajectory, queue_size=10)

    rate = rospy.Rate(10)

    dir_path = os.path.dirname(os.path.realpath(__file__))

    rospy.loginfo('obj_controller has started')

    path_runner = Main.Path_Runner()
    q_current = path_runner.get_current_q()

    while not rospy.is_shutdown():

        ind = input("press enter to continue")

        msg = JointTrajectory()

        msg_joint_names = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2',
                           'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']
        msg.joint_names = msg_joint_names
        point = JointTrajectoryPoint()

        q_current = path_runner.execute_step()
        msg_joint_positions = []
        for i in range(len(q_current)):
            for j in range(len(q_current[i])):
                msg_joint_positions.append(q_current[i][j])

        point.positions = msg_joint_positions
        msg.points = [point]

        rospy.Duration(0.1)
        pup.publish(msg)
        rate.sleep()

        # rospy.spinOnce()
