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
from datetime import datetime
from std_msgs.msg import String
import subprocess
from robotx_bionic_msgs.msg import TemperatureCurrent
from ina219 import INA219
from ina219 import DeviceRangeError
import time

SHUNT_OHMS = 0.1
ina = INA219(SHUNT_OHMS)
ina.configure()

class TempCurrentNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))

		self.temp_current_pub = rospy.Publisher('~temp_current', TemperatureCurrent, queue_size = 10)

	def cb(self, no_use):
		temp_cpu = self.get_cpu_temp()
		#current,power,shunt_vol = self.ina219_read()
		current, power, shunt_vol = self.ina219_read_samliu()
	        print datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		print "voltage: {:5.3f} mV ".format(shunt_vol)
		print "current: {:5.3f} mA".format(current)
	        print "  power: {:5.3f} mW".format(power)
		print "temperature on cpu: ", str(temp_cpu)
		print "---------------------"
		temp_press_msg = TemperatureCurrent()
		temp_press_msg.header.stamp = rospy.Time.now()
		#temp_press_msg.temperature = temp
		temp_press_msg.temperature_cpu = str(temp_cpu)
		temp_press_msg.power = str(power)
		temp_press_msg.current = str(current)
		temp_press_msg.voltage = str(shunt_vol)
		self.temp_current_pub.publish(temp_press_msg)
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
	
	def ina219_read(self):
	    ina_current =  -1
            power = -1
            shunt_voltage = -1
	    time.sleep(1)
	    #print("Bus Voltage: %.3f V" % ina.voltage())
	    try:
		ina_current =  ina.current()
		power=ina.power()
		shunt_voltage=ina.shunt_voltage()	
	        #print("Bus Current: %.3f mA" % ina_current)
	        #print("Power: %.3f mW" % ina.power())
	        #print("Shunt voltage: %.3f mV" % ina.shunt_voltage())
	    except DeviceRangeError as e:
		 # Current out of device range with specified shunt resister
	        print(e)
		ina.configure()
		time.sleep(1)
	    return ina_current,power,shunt_voltage


	def ina219_read_samliu(self):
            try:
                i =  ina.current()
                p = ina.power()
                v = ina.supply_voltage()
            except DeviceRangeError as e:
                 # Current out of device range with specified shunt resister
                 i, v, p = -1, -1, -1
	         print(e)
            return i, p, v

	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
	rospy.init_node("temp_current_node", anonymous = False)
	temp_current_node = TempCurrentNode()
	rospy.on_shutdown(temp_current_node.onShutdown)
	rospy.Timer(rospy.Duration(2), temp_current_node.cb)  # 15
	rospy.spin()
