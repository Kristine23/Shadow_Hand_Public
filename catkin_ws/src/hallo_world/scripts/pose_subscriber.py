#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose


def pose_callback(msg):
    rospy.loginfo('x: %f, y: %f', msg.x, msg.y)


if __name__ == '__main__':
    rospy.init_node('pose_subscriber')
    rospy.loginfo('pose_subscriber has started')

    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("pose is runnign")

    rospy.spin()
