#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# Callback function for the turtle's position
def turtle_callback(msg):
    rospy.loginfo("Turtle subscriber@[%f, %f, %f]", msg.x, msg.y, msg.theta)

def main():
    # Initialize the ROS node
    rospy.init_node('turtlesimcontrol', anonymous=True)

    # Setup subscriber to receive the turtle's position
    rospy.Subscriber('/turtle1/pose', Pose, turtle_callback)

    # Setup publisher to control the turtle's velocity
    turtle_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Define a rate for the loop
    rate = rospy.Rate(100)

    # Create the Twist message for velocity
    my_vel = Twist()
    my_vel.linear.x = 0.5
    my_vel.angular.z = 1.0

    # Main loop to continuously publish velocity commands
    while not rospy.is_shutdown():
        # Publish velocity to control the turtle's movement
        turtle_pub.publish(my_vel)

#Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    main()
