#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

# Global variables to store the poses of the turtle
turtle1_pose = None
turtle2_pose = None
distance_pub = None

# Function to handle position update for both turtles
def turtle_callback(msg, topic_name):
	global turtle1_pose, turtle2_pose, distance_pub
	if topic_name == "/turtle1/pose":
		turtle1_pose = msg
	elif topic_name == "/turtle2/pose":
		turtle2_pose = msg
		

def publish_distance(pub):
	global turtle1_pose, turtle2_pose, distance_pub
	# Computation of the Euclidean distance
	distance = math.sqrt((turtle1_pose.x - turtle2_pose.x) ** 2 + (turtle1_pose.y - 				turtle2_pose.y) ** 2)
	# Publish the distance on a topic
	distance_pub.publish(distance)
	rospy.loginfo("Distance between turtles: %f", distance)
	

def main():
	global turtle1_pose, turtle2_pose, distance_pub
	
	# Initialize the ROS node
	rospy.init_node("assignment1_distance", anonymous=True)
	rate = rospy.Rate(10)
	# Initialize the publisher
	distance_pub = rospy.Publisher("/turtle_distance", Float32, queue_size=10)
	# Initialize the subscribers
	rospy.Subscriber("/turtle1/pose", Pose, turtle_callback, "/turtle1/pose")
	rospy.Subscriber("/turtle2/pose", Pose, turtle_callback, "/turtle2/pose")
	
	while not rospy.is_shutdown():
		if turtle1_pose and turtle2_pose:
			publish_distance(distance_pub)
		rate.sleep()
	

if __name__ == '__main__':
	main()

