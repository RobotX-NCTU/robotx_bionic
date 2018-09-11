#!/usr/bin/env python
# Read temperature data measured from DS18B20 through serial
# and publish as ROS topic 
# Editor: Sean, Lu
# Last update: 3/31, 2018
#'''
#    Update: add port as parameter (3/30, 2018)
#    Update: add try except, use timer (3/31, 2018)
#'''
import rospy
import serial
from std_msgs.msg import String

class TempPressSerialNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		port = rospy.get_param("~port", "/dev/ttyUSB0")
		self.ard = serial.Serial(port, 9600)

		self.temp_press_pub = rospy.Publisher('~temp_press', String, queue_size = 10)

	def cb(self, no_use):
		res_str = self.ard.readline()
		print res_str
		temp_press_msg = String()
		temp_press_msg.data = res_str
		self.temp_press_pub.publish(temp_press_msg)

	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
	rospy.init_node("temp_press_serial_node", anonymous = False)
	temp_press_serial_node = TempPressSerialNode()
	rospy.on_shutdown(temp_press_serial_node.onShutdown)
	rospy.Timer(rospy.Duration.from_sec(1.0), temp_press_serial_node.cb)
	rospy.spin()