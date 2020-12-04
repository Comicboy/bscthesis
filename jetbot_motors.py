#!/usr/bin/env python3
import time
from jetbot.jetbot import Robot
import rospy
from std_msgs.msg import String


# stops all motors
def all_stop():
	robot.stop()


# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		robot.left()
	elif msg.data.lower() == "right":
		robot.right() 
	elif msg.data.lower() == "forward":
		robot.forward()
	elif msg.data.lower() == "backward":
		robot.backward()  
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# initialization
if __name__ == '__main__':

	# setup motor controller
	robot = Robot()

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()
