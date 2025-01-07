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

    dir_path = os.path.dirname(os.path.realpath(__file__))

    f_start = 'sh_small_movements_start.txt'
    f_move = 'sh_small_movements_movement.txt'

    file_start = dir_path + '/' + f_start
    file_move = dir_path + '/' + f_move

    file_start = './catkin_ws/test_q_sh/scripts/' + f_start
    file_move = './catkin_ws/test_q_sh/scripts/' + f_move

    with open(file_start) as f:
        lines = [line.rstrip() for line in f]

    with open(file_move) as f:
        line_move = [line.rstrip() for line in f]

    idx = 0
    idx_add = 1
    spondage_added = False

    while not rospy.is_shutdown():
        if idx >= len(lines)-idx_add and not spondage_added:
            ind = input("insert spondage. Press r when ready?")
            if ind == 'r':
                idx = 0
                lines = line_move
                spondage_added = True

        msg = JointTrajectory()

        fingers_idx = [1, 2, 3, 4, 0]
        print(idx)
        config = lines[idx]
        if idx + idx_add < len(lines):
            idx += idx_add
        arrays = array_from_sting(config)
        # print('arrays', type(arrays[0]))

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
        rospy.Duration(0.1)
        point.positions = msg_joint_positions
        msg.points = [point]
        pup.publish(msg)
        rate.sleep()
