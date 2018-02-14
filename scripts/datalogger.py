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

from sensor_msgs.msg import *

class Datalogger:
	def __init__(self):
		rospy.init_node("datalogger")

	def main(self):
		# Open subprocess to record bagfile with LiDAR topic
		rospy.loginfo(rospy.get_caller_id() + ": Start recording bag file...")
		proc = subprocess.Popen(['rosbag', 'record', '/scan'], preexec_fn=os.setsid, cwd='/home/ubuntu/bagfiles')

        #def shutdownCallback(*args):
	    #	proc.kill()
	    #	rospy.loginfo("Logger exiting")
        #signal.signal(signal.SIGINT, shutdownCallback)

		# Wait for user interrupt
		signal.pause()

		# End process using Ctrl+C and kill process
		rospy.loginfo(rospy.get_caller_id() + ": Stop recording bag file...")
		proc.send_signal(subprocess.signal.SIGINT)
		os.killpg(proc.pid, signal.SIGINT)
      

if __name__ == '__main__':
	logger = Datalogger()
	logger.main()
