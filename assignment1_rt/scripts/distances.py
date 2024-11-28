#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

# Initialization of global variables
turtle1_pose = None
turtle2_pose = None
turtle1_teleport = None
turtle2_teleport = None

# Function to handle position update for both turtles
def turtle_callback(msg, topic_name):

	global turtle1_pose, turtle2_pose
	
	if topic_name == "/turtle1/pose":
		turtle1_pose = msg
	elif topic_name == "/turtle2/pose":
		turtle2_pose = msg
		
	
# Function to stop a turtle
def stop_turtle(pub):
	stop_vel = Twist()
	pub.publish(stop_vel)
	

# Check boundaries and teleport in the grid
def check_and_teleport():

	global turtle1_pose, turtle2_pose, turtle1_teleport, turtle2_teleport, turtle1_pub, turtle2_pub
	
	# Boundaries
	x_min, x_max = 1.0, 10.0
	y_min, y_max = 1.0, 10.0

	# Check turtle1's position
	if turtle1_pose:
		if turtle1_pose.x < x_min or turtle1_pose.x > x_max or turtle1_pose.y < y_min or turtle1_pose.y > y_max:
			# Compute the new position on the border
			new_x = min(max(turtle1_pose.x, x_min), x_max)
			new_y = min(max(turtle1_pose.y, y_min), y_max)
			turtle1_teleport(new_x, new_y, turtle1_pose.theta)
			rospy.loginfo("Teletrasportata Turtle 1 sul bordo: (%.2f, %.2f)", new_x, new_y)
			stop_turtle(turtle1_pub)

	# Check turtle2's position
	if turtle2_pose:
		if turtle2_pose.x < x_min or turtle2_pose.x > x_max or turtle2_pose.y < y_min or turtle2_pose.y > y_max:
			# Compute the new position on the border
			new_x = min(max(turtle2_pose.x, x_min), x_max)
			new_y = min(max(turtle2_pose.y, y_min), y_max)
			turtle2_teleport(new_x, new_y, turtle2_pose.theta)
			rospy.loginfo("Teletrasportata Turtle 2 sul bordo: (%.2f, %.2f)", new_x, new_y)
			stop_turtle(turtle2_pub)


def main():
	global turtle1_teleport, turtle2_teleport, turtle1_pub, turtle2_pub

	# Initialize the ROS node
	rospy.init_node("assignment1_distance", anonymous=True)
	rate = rospy.Rate(100)

	# Initialize teleport services
	rospy.wait_for_service('/turtle1/teleport_absolute')
	rospy.wait_for_service('/turtle2/teleport_absolute')
	# Proxies for teleport services
	turtle1_teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	turtle2_teleport = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)
	
	# Initialize the publishers for turtle velocities
	turtle1_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1)
	turtle2_pub = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size = 1)
	# Initialize the subscribers for both turtles
	rospy.Subscriber("/turtle1/pose", Pose, turtle_callback, "/turtle1/pose")
	rospy.Subscriber("/turtle2/pose", Pose, turtle_callback, "/turtle2/pose")

	while not rospy.is_shutdown():
		if turtle1_pose and turtle2_pose:
			check_and_teleport()
		rate.sleep()

if __name__ == '__main__':
	main()

