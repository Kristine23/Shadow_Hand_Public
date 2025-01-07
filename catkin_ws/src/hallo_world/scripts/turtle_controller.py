#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

right = True


def call_set_pen_service(r, g, b, width, off):
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        response = set_pen(r, g, b, width, off)
        # rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call failed: {e}")


def pose_callback(msg: Pose):

    cmd = Twist()

    cmd.linear.x = 1.0
    cmd.angular.z = 1.0
    rospy.loginfo(cmd)
    pub.publish(cmd)

    global right

    if msg.x > 5.5 and not right:
        rospy.loginfo("Changing color to green")
        call_set_pen_service(0, 255, 0, 5, 0)
        right = True
    elif msg.x < 5.5 and right:
        rospy.loginfo("Changing color to red")
        right = False
        call_set_pen_service(255, 0, 0, 5, 0)


if __name__ == '__main__':
    rospy.init_node('turtle_controller')
    rospy.loginfo('turtle_controller has started')

    rospy.wait_for_service("/turtle1/set_pen")

    call_set_pen_service(255, 0, 0, 5, 0)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)

    rospy.spin()
