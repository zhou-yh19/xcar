#!/usr/bin/env python

# Monitor the racecar's battery level

import roslib
import rospy
from vesc_msgs.msg import VescStateStamped 

class racecar_battery():

	def __init__(self):
		rospy.init_node("racecar_battery")		

		#monitor racecar's battery 
		sub = rospy.Subscriber("/vesc/sensors/core", VescStateStamped, self.PowerEventCallback, queue_size=1)
		rospy.spin()


	def PowerEventCallback(self, msg):
		
		print("voltage: " + str(msg.state.voltage_input) + "  V") 
		
		if(msg.state.voltage_input < 10.8):
			print("\033[31;01mPlease charge!!!")

		print("\033[37;01m-----")

		rospy.sleep(5)
		
if __name__ == '__main__':
	try:
		racecar_battery()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")
