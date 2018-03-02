#!/usr/bin/env python
#
# Node publishing if push button is pressed or not (Bool)

import roslib
import rospy
import wiringpi

# Import messages
from std_msgs.msg import Bool

class Flagbutton:
	def __init__(self):
		# Create publisher
		self.__pub = rospy.Publisher('flagbutton_pressed', Bool, queue_size = 100)

		rospy.init_node('flagbutton')
		
		# Button GPIO number
		self.__GPIO_NO_BTN = 23

		# WiringPi setup
		wiringpi.wiringPiSetupGpio()
		rospy.loginfo(rospy.get_caller_id() + ': Wiring Pi GPIO initialized')
		
		# Define input on GPIO pin using pull up
		wiringpi.pinMode(self.__GPIO_NO_BTN, wiringpi.GPIO.INPUT)
		wiringpi.pullUpDnControl(self.__GPIO_NO_BTN, wiringpi.GPIO.PUD_UP)
		rospy.loginfo(rospy.get_caller_id() + ': Wiring Pi button pin set up')

	def main(self):
		# Loop at 10 Hz and publish message
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Publish True (button pressed) or False 
			self.__pub.publish(wiringpi.digitalRead(self.__GPIO_NO_BTN) == 0)
			rate.sleep()
	

      
if __name__ == '__main__':
	flagbutton = Flagbutton()
	flagbutton.main()

