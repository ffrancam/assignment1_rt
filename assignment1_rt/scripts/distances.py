#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

# Initialization of global variables
turtle1_pose = None
turtle2_pose = None
turtle1_pub = None
turtle2_pub = None

# Function to handle position update for both turtles
def turtle_callback(msg, topic_name):
	global turtle1_pose, turtle2_pose, turtle1_pub, turtle2_pub
	if topic_name == "/turtle1/pose":
		turtle1_pose = msg
	elif topic_name == "/turtle2/pose":
		turtle2_pose = msg
		

def publish_distance(pub):
	global turtle1_pose, turtle2_pose, turtle1_pub, turtle2_pub
	# Computation of the Euclidean distance
	distance = math.sqrt((turtle1_pose.x - turtle2_pose.x) ** 2 + (turtle1_pose.y - 				turtle2_pose.y) ** 2)
	# Publish the distance on a topic
	pub.publish(distance)
	rospy.loginfo("Distance between turtles: %f", distance)
	
	# If the moving turtle gets to close to the other one, it stops
	if distance < 1:
		stop_turtle()
		
		
def stop_turtle():
	global turtle1_pose, turtle2_pose, turtle1_pub, turtle2_pub
	
	# Stop the moving turtle
	stop_vel = Twist()
	
	if turtle1_pub:
		turtle1_pub.publish(stop_vel)
		rospy.loginfo("Turtle 1 is too close to turtle 2.")
	if turtle2_pub:
		turtle2_pub.publish(stop_vel)
		rospy.loginfo("Turtle 2 is too close to turtle 1.")
	

def main():
	global turtle1_pose, turtle2_pose, turtle1_pub, turtle2_pub
	
	# Initialize the ROS node
	rospy.init_node("assignment1_distance", anonymous=True)
	rate = rospy.Rate(10)
	# Initialize the distance publisher
	distance_pub = rospy.Publisher("/turtle_distance", Float32, queue_size = 1)
	# Initialize the publishers for turtle velocities
	turtle1_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1)
	turtle2_pub = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size = 1)
	# Initialize the subscribers for both turtles
	rospy.Subscriber("/turtle1/pose", Pose, turtle_callback, "/turtle1/pose")
	rospy.Subscriber("/turtle2/pose", Pose, turtle_callback, "/turtle2/pose")
	
	while not rospy.is_shutdown():
		if turtle1_pose and turtle2_pose:
			publish_distance(distance_pub)
		rate.sleep()
	

if __name__ == '__main__':
	main()

