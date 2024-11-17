#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

def turtlecallback(msg):
    rospy.loginfo("Turtle subscriber@[%f, %f, %f]", msg.x, msg.y, msg.theta)

def listener():
    rospy.init_node('turtlesimlistener', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, turtlecallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
