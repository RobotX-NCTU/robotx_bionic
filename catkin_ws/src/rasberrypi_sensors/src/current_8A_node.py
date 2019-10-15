#!/usr/bin/env python
# -*- coding: utf-8 -*-

#'''
# Editor :Lili Chiu 
# Last update : 2019/10/15 by lili
 
# This file is modified from the file get_voltage_current_power.py from the github repo 'DFRobot_INA219'
# Read current, voltage, power from the sensor and publish msg

#'''

#'''
#SEN0291 Wattmeter Sensor
#This sensor can detect Voltage ,Current,and Power
#The module has four I2C, these addresses are:

#ADDRESS 1  0x40   A0 = 0  A1 = 0
#ADDRESS 2  0x41   A0 = 1  A1 = 0
#ADDRESS 3  0x44   A0 = 0  A1 = 1
#ADDRESS 4  0x45   A0 = 1  A1 = 1

#Copyright    [DFRobot](http://www.dfrobot.com), 2016
#Copyright    GNU Lesser General Public License
#version  V1.0
#date  2019-2-27
#'''

import rospy
import serial
from std_msgs.msg import String
import subprocess
from robotx_bionic_msgs.msg import Current

import time
from DFRobot_INA219 import INA219

ina219_reading_mA = 1000
ext_meter_reading_mA = 1000

class CurrentNode_8A(object):
	def __init__(self):
		self.node_name = rospy.get_name()
                rospy.loginfo("[%s] Initializing " %(self.node_name))
		
		# addr: The sensor address 
		self.addr = rospy.get_param("~addr", "A1")
		if(self.addr == "A1"):
			self.ina = INA219(1, INA219.INA219_I2C_ADDRESS1)
			rospy.loginfo("Current sensor address : 0x40")
		else:
			self.addr = "A4"
			self.ina = INA219(1, INA219.INA219_I2C_ADDRESS4)
			rospy.loginfo("Current sensor address : 0x45" )
		while not self.ina.begin():
		    time.sleep(2)			#begin return True if succeed, otherwise return False
		#'''
		#Revise the following two paramters according to actula reading of the INA219 and the multimeter
		#for linearly calibration
		#'''
		self.ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)
                self.current_pub = rospy.Publisher('~current', Current, queue_size = 10)

        def cb(self, no_use):
		current, power, voltage = self.ina219_read()
                # print("sensor port", self.addr)
		# print datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                # print "voltage: {:5.3f} mV ".format(voltage)
                # print "current: {:5.3f} mA".format(current)
                # print "  power: {:5.3f} mW".format(power)
                # print "---------------------"
                current_msg = Current()
                current_msg.header.stamp = rospy.Time.now()
                current_msg.power = str(power)
                current_msg.current = str(current)
                current_msg.voltage = str(voltage)
                self.current_pub.publish(current_msg)

	
	def ina219_read(self):
            i = -1
	    p = -1
	    v = -1
	    i =  self.ina.get_current_mA()
            p = self.ina.get_power_mW()
            v = self.ina.get_bus_voltage_V()
            return i, p, v

        def onShutdown(self):
                rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
	nodename = "current_8A_node" 
        rospy.init_node(nodename, anonymous = False)
        current_node = CurrentNode_8A()
        rospy.on_shutdown(current_node.onShutdown)
        rospy.Timer(rospy.Duration(2), current_node.cb)  # 15
        rospy.spin()

