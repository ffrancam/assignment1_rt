#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, TeleportAbsolute

# Let the user select a turte to move
def select_turtle():
	global pub
	
	while True:
		print("\n")
		selected_turtle = input("Which turtle whould you like to move? Select (1) or (2):")
		# Define the publisher for the selected turtle
		if selected_turtle == "1":
			pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
			return pub
		elif selected_turtle == "2":
			pub = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=1)
			return pub
		else:
			print("\n")
			print("Please insert a valid input.")
			
			
def move_turtle()


def main():
    global pub, sub
    
    # Initialize the node
    rospy.init_node("Assignment1_UI", anonymous=True)
    rate = rospy.Rate(100)
    
    # Spawn turtle2 using the service client
    rospy.wait_for_service("/spawn")
    client1_spawn = rospy.ServiceProxy("/spawn", Spawn)
    client1_spawn(5.5, 4.0, 0.0, "turtle2")
    
    while not rospy.is_shutdown():
    	pub = select_turtle()
    	move_turtle(pub)
    	rate.sleep()
    	


if __name__ == '__main__':
    main()

