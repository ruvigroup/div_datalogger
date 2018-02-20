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
		self.__STATE_ID_IDLE		= 0
		self.__STATE_ID_REC		= 1
		self.__STATE_ID_SHTDN		= 10

		# Request ID's
		self.__REQ_ID_STATUS		= 0
		self.__REQ_ID_REC_START		= 1
		self.__REQ_ID_REC_STOP		= 2
		self.__REQ_ID_SYS_SHTDN		= 10

		# Response message ID's
		self.__RESP_MSG_ID_SUCCESS 	= 0
		self.__RESP_MSG_ID_FAILED 	= -1

		# Sleep time to wait for subprocess response [sec]
		self.__PROC_RESP_SLEEPTIME	= 1

		# Variables
		self.__rec_time_start	= rospy.Time(secs = 0, nsecs = 0)
		self.__state_id		= self.__STATE_ID_IDLE
		self.__proc_rec 	= []


	def __handle_user_cmd(self, request):
		if request.request_id == self.__REQ_ID_STATUS:
			# Case when invalid request id is used
			rospy.loginfo(rospy.get_caller_id() + ': Status requested')
			msg_id = 0
			rec_time = self.__rec_time_start

		elif request.request_id == self.__REQ_ID_REC_START:
			if self.__state_id == self.__STATE_ID_IDLE:
				# Prepare log message
				rospy.loginfo(rospy.get_caller_id() + ': Start recording bag file...')
				# Open sub process for bagrecord (LZ4 compressed)
				self.__proc_rec = subprocess.Popen(['rosbag', 'record', '--lz4', '/imu/data'], preexec_fn=os.setsid, cwd='/home/ubuntu/bagfiles')
				# Save log start time
				self.__rec_time_start = rospy.Time.now()
				# Return save log start time
				rec_time = self.__rec_time_start			
				# Check if process call was successful
				if self.__proc_rec.poll() == None:
					# Change state
					self.__state_id = self.__STATE_ID_REC
					msg_id = 0		
				else:
					rospy.loginfo(rospy.get_caller_id() + ': No valid request ID.')
					msg_id = -1
			else:
				rospy.loginfo(rospy.get_caller_id() + ': No valid request ID.')
				msg_id = -1
				rec_time = rospy.Time(secs = 0, nsecs = 0)

		elif request.request_id == self.__REQ_ID_REC_STOP:
			if self.__state_id == self.__STATE_ID_REC:
				# Prepare response and log message
				rospy.loginfo(rospy.get_caller_id() + ': Stop recording bag file...')
				# Calculate final recording time
				rec_time = rospy.Time.now() - self.__rec_time_start
				# Abort subprocess and kill process in os
				self.__proc_rec.send_signal(subprocess.signal.SIGINT)
				os.killpg(self.__proc_rec.pid, signal.SIGINT)
				# Check if process call was successful
				rospy.sleep(.5)
				if self.__proc_rec.poll() == None:
					rospy.loginfo(rospy.get_caller_id() + ': No valid request ID.')
					msg_id = -1
				else:
					# Change state
					self.__state_id = self.__STATE_ID_IDLE
					msg_id = 0
			else:
				rospy.loginfo(rospy.get_caller_id() + ': No valid request ID.')
				msg_id = -1
				rec_time = rospy.Time(secs = 0, nsecs = 0)

		elif request.request_id == self.__REQ_ID_SYS_SHTDN:
			#if self.__state_id == self.__STATE_ID_IDLE:
				# Change state
			self.__state_id = self.__STATE_ID_SHTDN
			rospy.loginfo(rospy.get_caller_id() + ': Shutting down system...')
			subprocess.Popen(['shutdown', 'now'])
			msg_id = 0
			#else:
			#	rospy.loginfo(rospy.get_caller_id() + ': No valid request ID.')
			#	msg_id = -1
			rec_time = rospy.Time(secs = 0, nsecs = 0)

		else:
			# Case when invalid request id is used
			rospy.loginfo(rospy.get_caller_id() + ': No valid request ID.')
			msg_id = -1
			rec_time = rospy.Time(secs = 0, nsecs = 0)

		return UserCmdResponse(self.__state_id, msg_id, rec_time)


	def main(self):
		# Call UserCmd service
		service_UserCmd = rospy.Service('user_cmd', UserCmd, self.__handle_user_cmd)
		
		# Keep ROS busy
		rospy.spin()

      
if __name__ == '__main__':
	logger = Datalogger()
	logger.main()

	
