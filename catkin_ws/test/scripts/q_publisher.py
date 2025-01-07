#!/usr/bin/env python3

import os

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def vector_from_string(string):
    array = []
    values = string.split(',')
    for v in values:
        array.append(float(v))
    return array


def array_from_sting(string):

    string.replace(" ", "")
    s = string.split("=", 1)
    arrays = s[-1]
    a = 0
    while arrays[a] != '[':
        a += 1
        if a > 100:
            print('array_from_sting not working, somethings wrong!')
            return
    n = 0
    while arrays[n+a] == '[':
        n += 1
    n -= 1

    # print(s[0], 'n', n)
    # print(arrays)
    # print('0', arrays[0], '1', arrays[1], '2', arrays[2])

    if n == 0:
        v = arrays.split('[')[-1]
        u = v.split(']')[0]
        return vector_from_string(u)

    res = []
    for i in range(n):
        res.append([])  # arrays outermost to inner most.

    depth = n-1

    elements = arrays.split("],[")

    for e in elements:
        v = e.split('[')[-1]
        if ']' not in e:
            depth = n-1
            vector = vector_from_string(v)
            res[depth].append(vector)
        else:
            u = v.split(']')
            v = u[0]
            vector = vector_from_string(v)

            res[depth].append(vector)
            c = len(u)-1

            while depth >= 1 and c > 0:
                res[depth-1].append(res[depth].copy())
                res[depth] = []
                c -= 1
                depth -= 1
    return res[0]


if __name__ == '__main__':
    rospy.init_node('test_q_values')
    rospy.loginfo('test_q_values')

    pup = rospy.Publisher("/rh_trajectory_controller/command",
                          JointTrajectory, queue_size=10)

    rate = rospy.Rate(10)

    f_0 = 'f0.txt'
    f_1 = 'f1.txt'
    f_2 = 'f2.txt'
    f_3 = 'f3.txt'
    f_4 = 'f4.txt'
    f_01 = 'f01.txt'
    f_014 = 'f014.txt'
    f_0134 = 'f0134.txt'
    f_full = 'f_full.txt'

    dir_path = os.path.dirname(os.path.realpath(__file__))

    file_name = dir_path + '/' + f_full
    file_name = dir_path + '/' + f_0134
    file_name = dir_path + '/' + f_014
    file_name = dir_path + '/' + f_01
    file_name = dir_path + '/' + f_2
    file_name = dir_path + '/' + f_3
    file_name = dir_path + '/' + f_4
    file_name = dir_path + '/' + f_1
    file_name = dir_path + '/' + f_0

    file_name = './catkin_ws/test/scripts/' + f_01

    with open(file_name) as f:
        lines = [line.rstrip() for line in f]

    print('number_of_lines', len(lines))
    idx = 0
    idx_add = 1

    while not rospy.is_shutdown():
        msg = JointTrajectory()

        fingers_idx = [1, 2, 3, 4, 0]

        config = lines[idx]
        if idx + idx_add < len(lines):
            idx += idx_add
        arrays = array_from_sting(config)
        # print('arrays', type(arrays[0]))

        if idx % 2 == 0:
            ind = input("press button to continue")

        fingers_joint_names = [['rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5'], ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4'], [
            'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4'], ['rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4'], ['rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5']]

        msg_joint_names = []
        msg_joint_positions = []
        for i in fingers_idx:
            msg_joint_names += fingers_joint_names[i]
            new_array = arrays[i]
            if i == 3:
                new_array[0] = -new_array[0]
            if i == 4:
                new_array[1] = -new_array[1]
            reversed_array = new_array[::-1]

            msg_joint_positions += reversed_array

        # msg.joint_names = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2',
        #                   'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']
        msg.joint_names = msg_joint_names
        point = JointTrajectoryPoint()

        point.positions = msg_joint_positions
        msg.points = [point]
        rospy.Duration(0.1)
        pup.publish(msg)
        rate.sleep()
