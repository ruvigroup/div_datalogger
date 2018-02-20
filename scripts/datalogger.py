#!/usr/bin/env python
#
# State machine:
#  IDLE
#  LOGGING
#  STREAMING
#  (PAUSE?)
#
# Sub states:
#  -
#
# Input topics:
#  /scan - laser scan topic
#
# Parameters:
# -

import roslib #; manifest?
import rospy
import os
import subprocess
import signal
# import roslaunch	

# Import messages
from sensor_msgs.msg import *

# Import services
from div_datalogger.srv import *


class Datalogger:
	def __init__(self):
		rospy.init_node("datalogger")
		
		# State ID's
		self.__STATE_ID_IDLE	= 0
		self.__STATE_ID_REC	= 1
		self.__STATE_ID_SHTDN	= 10

		self.__REQ_ID_REC_START	= 1
		self.__REQ_ID_REC_STOP	= 2
		self.__REQ_ID_SYS_SHTDN	= 10

		self.__rec_time_start	= None	
		self.__state		= self.__STATE_ID_IDLE

		self.__proc_rec 	= []


	def __handleUserCmd(self, request):			
		if request.request_id == self.__REQ_ID_REC_START:
			# Change state
			self.__state = self.__STATE_ID_REC
			# Prepare response and log message			
			msg = (rospy.get_caller_id() + ": Start recording bag file...")
			rospy.loginfo(msg)
			# Open sub process for bagrecord (LZ4 compressed)
			self.__proc_rec = subprocess.Popen(['rosbag', 'record', '--lz4', '/imu/data'], preexec_fn=os.setsid, cwd='/home/ubuntu/bagfiles')
			# Save log start time
			self.__rec_time_start = rospy.Time.now()
			# Recording time zero initially
			rec_time = rospy.Time(secs = 0, nsecs = 0)

		elif request.request_id == self.__REQ_ID_REC_STOP:
			# Change state
			self.state = self.__STATE_ID_IDLE
			# Prepare response and log message
			msg = (rospy.get_caller_id() + ": Stop recording bag file...")
			rospy.loginfo(msg)
			# Calculate final recording time
			rec_time = rospy.Time.now() - self.__rec_time_start
			# Abort subprocess and kill process in os
			self.__proc_rec.send_signal(subprocess.signal.SIGINT)
			os.killpg(self.__proc_rec.pid, signal.SIGINT)

		elif request.request_id == self.__REQ_ID_SYS_SHTDN:
			# Change state
			self.state = self.__STATE_ID_SHTDN
			msg = (rospy.get_caller_id() + ": Shutting down system...")
			subprocess.Popen(['shutdown', 'now'])

		else:
			# Case when invalid request id is used
			msg = (rospy.get_caller_id() + ": No valid request ID.")
			rec_time = rospy.Time(secs = 0, nsecs = 0)

		return UserCmdResponse(self.__state, msg, rec_time) 

	def main(self):
		# Call UserCmd service
		service_UserCmd = rospy.Service('UserCmd', UserCmd, self.__handleUserCmd)
		
		# Keep ROS busy
		rospy.spin()

      
if __name__ == '__main__':
	logger = Datalogger()
	logger.main()

	
