#!/usr/bin/env python3
import time
from jetbot.jetbot import Robot
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

goal_x = 0.0
goal_y = 0.0
goal_angle = 0.0

# Stops all motors
def all_stop():
	robot.stop()

def goalCallback(msg):
	# Getting the quaternion representing the orientation of the goal
	orientation_q = msg.pose.pose.orientation
	
	# Creating a list which contains each element of the quaternion for conversion
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	
	# Converting the orientation from quaternion to euler
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	goal_angle = yaw

def poseCallback(msg):
	# Getting the quaternion representing the current orientation of the robot
	orientation_q = msg.pose.pose.orientation
	
	# Creating a list which contains each element of the quaternion for conversion
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	
	# Converting the orientation quaternion to euler
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	current_angle = yaw
	
	# Rounding the orientations in order to counter the lack of accuracy of the motor controls
	goal_angle = round(goal_angle, 1)
	current_angle = round(current_angle, 1)
	
	#
	if(current_angle != goal_angle):
		if(current_angle < goal_angle):
			while(current_angle < goal_angle):
				robot.left(0.5)
			robot.forward(0.5)
		else:
			while(current_angle > goal_angle):
				robot.right(0.5)
			robot.forward(0.5)
	else:
		robot.forward(0.5)

	
	
# Handle simple string commands from the server (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		robot.left(0.5)
		time.sleep(3)
	elif msg.data.lower(0.5) == "right":
		robot.right(0.5)
		time.sleep(3)
	elif msg.data.lower() == "forward":
		robot.forward(0.5)
		time.sleep(3)
	elif msg.data.lower() == "backward":
		robot.backward(0.5)
		time.sleep(3)
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
