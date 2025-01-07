#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('draw_circle')
    rospy.loginfo('draw_circle has started', rospy.get_time())

    pup = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        pup.publish(msg)
        rate.sleep()
