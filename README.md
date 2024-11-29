# Assignment 1 - Turtle Control and Collision Avoidance System

## **🐢 Overview**

This project consists of two nodes designed to manage and control two turtles in a 'turtlesim' environment using ROS (Robot Operating System). Its main functionalities are:
- Controlling turtle movement through user inputs in a UI,
- Monitoring the distance between the two turtles and ensuring a safe distance,
- Preventing the turtles from moving out of a defined bounded space.



## **Getting Started**

### 1. Install ROS
In order to run the project, ROS and turtlesim package must be installed.

### 2. Clone the repository 
Clone the repository into the src folder of your ROS workspace:

```bash
git clone <URL_of_this_repository>
```

### 3. Run ROS Master
Before running the scripts, it's necessary to start the ROS master node:

```bash
roscore
```

### 4. Start turtlesim
In a second terminal, go into your workspace and start the turtlesim simulation node:

```bash
rosrun turtlesim turtlesim_node
```
### 5. Run the UI.py and distances.py scripts
After starting the turtlesim node, open two new terminals and go into your workspace once again in both of them. In one, start the UI node
```bash
rosrun assignment1_rt UI.py
```
and in the other start the distances node
```bash
rosrun assignment1_rt distances.py
```
Thanks to the user inface implemented in UI.py, through the terminal where you started the node you can now control the two turtles.


## How it works

### • UI.py
This script provides a command-line interface for controlling two turtles. Its main features are:
#### - Turtle Spawining
The node uses the /spawn service to create a new turtle (turtle2) positioned in a specified position in the turtlesim environment.
#### - User Interaction
The user is prompted to select a turtle to control and in order to move the selected turtle, the values of the velocities (linear x, linear y, angular z) must be specified.
#### - Velocity Publishing
By using a Twist message initialized with the velocities given via input and publishing it to the turtle's /cmd_vel topic, the turtle is able to move. After 1 second, a zero velocity command is sent to stop it.
#### - Iterative Control
If a non valid input is given or the turtle has finished its movement, the user is able to choose again.

### • distances.py
This script monitors and manages the interactions and positions of the two turtles. It automates the collision avoidance and compliance with boundaries. Its main features are:
#### - Position tracking
The node tracks the positions and the velocities of the turble by using a ROS subscribers.
#### - Proximity between the turtles
The code ensures that the turtles maintain a minimum distance of 1 unit by using the teleport service. To do so, checks are performed on the distance (computed using the Euclidean norm) and as soon as it falls below the imposed limit, a check on the velocities is done to understand which turtle needs to be stopped. Subsequently, a new "safe" position along the turtle's trajectory is computed and the teleport is called. 
#### - Boundaries on the area
The turtles are kept in a "safe" area ( 1 <= x <= 10 and 1 <= y <= 10 ) by teleporting them into a valid position when necessary.
