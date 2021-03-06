#!/usr/bin/env python
import math
import time
from jetbot import Robot
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# Global variables for the robots current 2D position and orientation

# Stops all motors
def all_stop():
	robot.stop()

def goalCallback(msg):
	print('I received the goal coordinates!')
        global goal_x
        global goal_y
	# Getting the 2D coordinates of the goal
	goal_x = msg.pose.position.x
	goal_y = msg.pose.position.y

def poseCallback(msg):
	print('I got the position!')
        global goal_x
        global goal_y
	
	# Getting the current 2D coordinates of the robots position
	current_x = msg.pose.position.x
	current_y = msg.pose.position.y
	
	# Getting the quaternion representing the current orientation of the robot
	orientation_q = msg.pose.orientation
	
	# Creating a list which contains each element of the quaternion for conversion
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	
	# Converting the orientation quaternion to euler
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	current_angle = yaw
	
	# Rounding the orientations and positions in order to counter the lack of accuracy of the motor controls
	goal_x = round(goal_x, 1)
	goal_y = round(goal_y, 1)
	
	current_x = round(current_x, 1)
	current_y = round(current_y, 1)
	current_angle = round(current_angle, 1)
	
	# Calculating the angle in which we have to turn in order to reach the goal
	target_angle = math.atan((goal_y - current_y)/(goal_x - current_x))
	target_angle = round(target_angle, 1)
	
	print('Goal: ', goal_x, goal_y)
	print('Position: ', current_x, current_y, current_angle)
	print('Target angle: ', target_angle)
	
	# If we are at the goal coordinates we stop, otherwise we move towards its direction by first turning into the correct angle (target_angle) then moving forward
	if(current_x != goal_x or current_y != goal_y):
		if(current_angle != target_angle):
			if(current_angle < target_angle):
				print('Im going left')
				robot.left(0.1)
			else:
				print('Im going right')
				robot.right(0.1)
		else:
			print('Im going forward')
			robot.forward(0.1)
	else:
		print('Im stopping')
		all_stop()

	
	
# Handle simple string commands from the server (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		robot.left(0.1)
	elif msg.data.lower() == "right":
		robot.right(0.1)
	elif msg.data.lower() == "forward":
		robot.forward(0.1)
	elif msg.data.lower() == "backward":
		robot.backward(0.1)
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# Initialization
if __name__ == '__main__':

	# Setup motor controller
	robot = Robot()

	# Stop the motors as precaution
	all_stop()

        goal_x = 0.0
        goal_y = 0.0

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	# Subscribe to the relevant topics
	rospy.Subscriber('move_base_simple/goal', PoseStamped, goalCallback) # for getting the navigation goal of the robot
	rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, poseCallback) # for getting the current position of the robot from the slam
	rospy.Subscriber('~cmd_str', String, on_cmd_str) # for getting collision avoidance commands from the server

	# Start running
	rospy.spin()

	# Stop motors before exiting
	all_stop()
