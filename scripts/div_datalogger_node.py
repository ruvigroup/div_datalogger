#!/usr/bin/env python3
#
# Node subscribing to sensor topics and handling user request via service
#
# State machine:
#  IDLE
#  LOGGING
#  SHUTDOWN
#
# Subscribed topics:
#  come as parameter from launch file
#
# Service:
#  /user_cmd
# 	-> Request:		ID
#	-> Response:	State ID, Message ID, Recording time

import roslib
import rospy
import os
import subprocess
import signal

# Import messages
from sensor_msgs.msg import *

# Import services
from div_datalogger.srv import *


class Datalogger:
	def __init__(self, topic_list, internal_path, external_path):
		# Create name for node
		rospy.init_node("remote_bag_logging")

		# State ID's
		self.__STATE_ID_IDLE		= 0
		self.__STATE_ID_REC			= 1
		self.__STATE_ID_SHTDN		= 10

		# Request ID's
		self.__REQ_ID_STATUS		= 0
		self.__REQ_ID_REC_START		= 1
		self.__REQ_ID_REC_STOP		= 2
		self.__REQ_ID_SYS_SHTDN		= 10

		# Response message ID's
		self.__RESP_MSG_ID_SUCCESS 	= 0
		self.__RESP_MSG_ID_FAILED 	= -1

		# Sleep times for waiting for subprocess response [sec]
		self.__PROC_RESP_SLEEPTIME	= 1
		self.__PROC_KILL_SLEEPTIME	= 5

		# Variable initialization
		self.__rec_time_start	= rospy.Time(secs = 0, nsecs = 0)
		self.__state_id			= self.__STATE_ID_IDLE
		self.__proc_rec 		= []

		# List of topics to record
		self.topic_list = topic_list

		# Paths from launch file for bag file destination
		self.internal_path = internal_path
		self.external_path = external_path

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
				# Check if usb storage is attached
				if os.path.isdir(self.external_path):
					path_usb_drive = self.external_path
				else:
					path_usb_drive = self.internal_path

				# Open sub process for bagrecord
				command_list = ['rosbag', 'record', '--split', '--size=1024']
				command_list.extend(self.topic_list)
				self.__proc_rec = subprocess.Popen(command_list, preexec_fn=os.setsid, cwd=path_usb_drive)
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
					rospy.logerr(rospy.get_caller_id() + ': Recording subprocess not started')
					msg_id = -1
			else:
				rospy.logwarn(rospy.get_caller_id() + ': Invalid request, already recording')
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
				rospy.sleep(self.__PROC_KILL_SLEEPTIME) # leave enough time to be sure that the process is killed
				if self.__proc_rec.poll() == None:
					rospy.logerr(rospy.get_caller_id() + ': Recording subprocess not stopped')
					msg_id = -1
				else:
					# Change state
					self.__state_id = self.__STATE_ID_IDLE
					msg_id = 0
			else:
				rospy.logwarn(rospy.get_caller_id() + ': Invalid request, not recording')
				msg_id = -1
				rec_time = rospy.Time(secs = 0, nsecs = 0)


		elif request.request_id == self.__REQ_ID_SYS_SHTDN:
			# Change state
			self.__state_id = self.__STATE_ID_SHTDN
			rospy.loginfo(rospy.get_caller_id() + ': Shutting down system...')
			os.system('sudo shutdown now')
			msg_id = 0
			rec_time = rospy.Time(secs = 0, nsecs = 0)

		else:
			# Case when invalid request id is used
			rospy.loginfo(rospy.get_caller_id() + ': No valid request')
			msg_id = -1
			rec_time = rospy.Time(secs = 0, nsecs = 0)

		return UserCmdResponse(self.__state_id, msg_id, rec_time)


	def main(self):
		# Call UserCmd service
		service_UserCmd = rospy.Service('user_cmd', UserCmd, self.__handle_user_cmd)

		# Keep ROS busy
		rospy.spin()


if __name__ == '__main__':
	# Read paramters from launch file
	topic_list = rospy.get_param('remote_bag_logging/topic_list')
	internal_path = rospy.get_param('remote_bag_logging/int_rec_path')
	external_path = rospy.get_param('remote_bag_logging/ext_rec_path')

	# Initialize and start class
	logger = Datalogger(topic_list, internal_path, external_path)
	logger.main()
