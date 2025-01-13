#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist
import math

# Initialization of global variables
turtle1_pose = None
turtle2_pose = None
turtle1_teleport = None
turtle2_teleport = None
turtle1_velocity = Twist()
turtle2_velocity = Twist()
turtle1_obstacles = []
turtle2_obstacles = []

# Minimum safe distance from obstacles
SAFE_DISTANCE = 1.0

# Function to handle position update for both turtles
def turtle_callback(msg, topic_name):
	global turtle1_pose, turtle2_pose

	if topic_name == "/turtle1/pose":
		turtle1_pose = msg
	elif topic_name == "/turtle2/pose":
		turtle2_pose = msg


# Function to handle velocity update for both turtles
def velocity_callback(msg, turtle_id):
	global turtle1_velocity, turtle2_velocity
	
	if turtle_id == 1:
		turtle1_velocity = msg
	elif turtle_id == 2:
		turtle2_velocity = msg


# Function to stop a turtle
def stop_turtle(pub):
	stop_vel = Twist()
	pub.publish(stop_vel)


def check_distance():
	global turtle1_pose, turtle2_pose, turtle1_teleport, turtle2_teleport, turtle1_pub, turtle2_pub, turtle1_velocity, turtle2_velocity
	# Minimum distance between the two turtles
	min_distance = 1
	# Computation of the Euclidean distance
	distance = math.sqrt((turtle1_pose.x - turtle2_pose.x) ** 2 + (turtle1_pose.y - turtle2_pose.y) ** 2)
	
	if distance < min_distance:
		# If turtle 1's velocity is non-zero, stop it
		if (turtle1_velocity.linear.x != 0 or turtle1_velocity.linear.y != 0 or turtle1_velocity.angular.z != 0):
			stop_turtle(turtle1_pub)
			# Compute turtle's position and teleport
			new_x1, new_y1 = get_new_position(turtle1_pose, turtle2_pose, min_distance + 0.01)
			turtle1_teleport(new_x1, new_y1, turtle1_pose.theta)
			rospy.loginfo("Turtle 1 teleported to (%.2f, %.2f)", new_x1, new_y1)
		# If turtle 2's velocity is non-zero, stop it
		elif (turtle2_velocity.linear.x != 0 or turtle2_velocity.linear.y != 0 or turtle2_velocity.angular.z != 0):
			stop_turtle(turtle2_pub)
			# Compute turtle's position and teleport
			new_x2, new_y2 = get_new_position(turtle2_pose, turtle1_pose, min_distance + 0.1)
			turtle2_teleport(new_x2, new_y2, turtle2_pose.theta)
			rospy.loginfo(f"Turtle 2 teleported to (%.2f, %.2f)", new_x2, new_y2)


def get_new_position(turtle_pose, other_turtle_pose, min_distance):
	# Calculate the direction vector from the current turtle to the other turtle
	dx = other_turtle_pose.x - turtle_pose.x
	dy = other_turtle_pose.y - turtle_pose.y
	distance_to_other = math.sqrt(dx**2 + dy**2)

	# Normalize the direction vector
	dx /= distance_to_other
	dy /= distance_to_other

	# Compute the correct position
	new_x = other_turtle_pose.x - dx * min_distance
	new_y = other_turtle_pose.y - dy * min_distance

	return new_x, new_y


# Check boundaries and teleport in the grid
def check_and_teleport():
	global turtle1_pose, turtle2_pose, turtle1_teleport, turtle2_teleport, turtle1_pub, turtle2_pub
	
	# Boundaries
	x_min, x_max = 1.0, 10.0
	y_min, y_max = 1.0, 10.0

	# Check turtle1's position
	if turtle1_pose:
		if turtle1_pose.x < x_min or turtle1_pose.x > x_max or turtle1_pose.y < y_min or turtle1_pose.y > y_max:
			# Compute the new position on the border and teleport
			new_x = min(max(turtle1_pose.x, x_min), x_max)
			new_y = min(max(turtle1_pose.y, y_min), y_max)
			turtle1_teleport(new_x, new_y, turtle1_pose.theta)
			rospy.loginfo("Turtle 1 teleported on border: (%.2f, %.2f)", new_x, new_y)
			stop_turtle(turtle1_pub)

	# Check turtle2's position
	if turtle2_pose:
		if turtle2_pose.x < x_min or turtle2_pose.x > x_max or turtle2_pose.y < y_min or turtle2_pose.y > y_max:
			# Compute the new position on the border and teleport
			new_x = min(max(turtle2_pose.x, x_min), x_max)
			new_y = min(max(turtle2_pose.y, y_min), y_max)
			turtle2_teleport(new_x, new_y, turtle2_pose.theta)
			rospy.loginfo("Turtle 2 teleported on border: (%.2f, %.2f)", new_x, new_y)
			stop_turtle(turtle2_pub)
			
			
# Obstacles callback function
def obstacle_callback(msg, turtle_id):
	global turtle1_obstacles, turtle2_obstacles
	
	if turtle_id == 1:
		turtle1_obstacles = msg.data
	elif turtle_id == 2:
		turtle2_obstacles = msg.data
        
# Check distance to obstacles and stop if necessary
def check_obstacles(turtle_obstacles, turtle_pub):
	global SAFE_DISTANCE

	if any(distance < SAFE_DISTANCE for distance in turtle_obstacles):
		rospy.loginfo("Obstacle too close! Stopping turtle.")
		stop_turtle(turtle_pub)


def main():
	global turtle1_teleport, turtle2_teleport, turtle1_pub, turtle2_pub, turtle1_velocity, turtle2_velocity

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
	rospy.Subscriber("/turtle1/cmd_vel", Twist, lambda msg: velocity_callback(msg, 1))
	rospy.Subscriber("/turtle2/cmd_vel", Twist, lambda msg: velocity_callback(msg, 2))
	rospy.Subscriber("/turtle1/obstacles", Float32MultiArray, lambda msg: obstacle_callback(msg, 1))
	rospy.Subscriber("/turtle2/obstacles", Float32MultiArray, lambda msg: obstacle_callback(msg, 2))
	
	while not rospy.is_shutdown():
		if turtle1_pose and turtle2_pose:
			check_and_teleport()
			check_distance()
			check_obstacles(turtle1_obstacles, turtle1_pub)
			check_obstacles(turtle2_obstacles, turtle2_pub)
		rate.sleep()

if __name__ == '__main__':
	main()

