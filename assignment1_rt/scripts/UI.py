#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, TeleportAbsolute

def main():
    global pub1, pub2
    
    # Initialize the node
    rospy.init_node("Assignment1_UI", anonymous=True)
    
    # Spawn turtle2 using the service client
    rospy.wait_for_service("/spawn")
    client1_spawn = rospy.ServiceProxy("/spawn", Spawn)
    client1_spawn(5.5, 4.0, 0.0, "turtle2")
    
    # Define publishers for both turtles
    pub1 = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
    pub2 = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=1)


if __name__ == '__main__':
    main()

