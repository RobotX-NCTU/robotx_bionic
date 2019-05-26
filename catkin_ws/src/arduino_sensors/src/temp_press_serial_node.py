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
from robotx_bionic_msgs.msg import TemperaturePressure

class TempPressSerialNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		port = rospy.get_param("~port", "/dev/ttyUSB0")
		self.ard = serial.Serial(port, 9600)

		self.temp_press_pub = rospy.Publisher('~temp_press', TemperaturePressure, queue_size = 10)

	def cb(self, no_use):
		res_str = self.ard.readline()
		#i2 =  res_str.split(" ")[3].split("\r")[0]
		#i1 = res_str.split(" ")[2]
		temp_cpu = self.get_cpu_temp()
		temp =  res_str.split(" ")[1]
		press = res_str.split(" ")[0]
		print "temperature: ", temp
		"temperature on cpu: ", temp_cpu
		print "pressure: ", press
		#print "I1: ", i1
		#print "I2: ", i2
		print "---------------------"
		temp_press_msg = TemperaturePressure()
		temp_press_msg.header.stamp = rospy.Time.now()
		temp_press_msg.temperature = temp
		temp_press_msg.temperature_cpu = temp_cpu
		temp_press_msg.pressure = press
		#temp_press_msg.current1 = i1
		#temp_press_msg.current2 = i2
		self.temp_press_pub.publish(temp_press_msg)
		#temp_press_msg = String()
		#temp_press_msg.data = res_str
		#self.temp_press_pub.publish(temp_press_msg)
	
	def get_cpu_temp(self):
		# Call command like a command line shell and get the return value
		ret_byte = subprocess.check_output(['vcgencmd', 'measure_temp'])
		# Convert byte to string value, the result is like "temp=48.5'C"
		ret_str = ret_byte.decode('utf-8')
		# Cut string from 'equal symbol' to 'degree C symbol', then convert to float
		cpu_temp = float(ret_str[ret_str.find('=')+1: ret_str.find('\'')])
		return cpu_temp

	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
	rospy.init_node("temp_press_serial_node", anonymous = False)
	temp_press_serial_node = TempPressSerialNode()
	rospy.on_shutdown(temp_press_serial_node.onShutdown)
	rospy.Timer(rospy.Duration.from_sec(0.2), temp_press_serial_node.cb)
	rospy.spin()