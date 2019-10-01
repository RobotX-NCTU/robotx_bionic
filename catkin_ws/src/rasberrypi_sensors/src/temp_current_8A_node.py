#!/usr/bin/env python
# -*- coding: utf-8 -*-

#'''
#file get_voltage_current_power.py
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

import time
from DFRobot_INA219 import INA219

ina219_reading_mA = 1000
ext_meter_reading_mA = 1000

ina = INA219(1, INA219.INA219_I2C_ADDRESS4)                                #Change I2C address by dialing DIP switch

#begin return True if succeed, otherwise return False
while not ina.begin():
    time.sleep(2)
#'''
#Revise the following two paramters according to actula reading of the INA219 and the multimeter
#for linearly calibration
#'''
ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)

#ina.reset()                                     #Resets all registers to default values

class TempCurrentNode_8A(object):
        def __init__(self):
                self.node_name = rospy.get_name()
                rospy.loginfo("[%s] Initializing " %(self.node_name))

                self.temp_current_pub = rospy.Publisher('~temp_current', TemperatureCurrent, queue_size = 10)

        def cb(self, no_use):
                temp_cpu = self.get_cpu_temp()
                #current,power,shunt_vol = self.ina219_read()
                current, power, voltage = self.ina219_read()
                print datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                print "voltage: {:5.3f} mV ".format(voltage)
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
                temp_press_msg.voltage = str(voltage)
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
            i = -1
	    p = -1
	    v = -1
	    i =  ina.get_current_mA()
            p = ina.get_power_mW()
            v = ina.get_bus_voltage_V()
            return i, p, v

        def onShutdown(self):
                rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
        rospy.init_node("temp_current_8A_node", anonymous = False)
        temp_current_node = TempCurrentNode_8A()
        rospy.on_shutdown(temp_current_node.onShutdown)
        rospy.Timer(rospy.Duration(2), temp_current_node.cb)  # 15
        rospy.spin()

#def main():
#    while True:
#        time.sleep(1)
#        print ("Shunt Voltage : %.2f mV" % ina.get_shunt_voltage_mV())
#        print ("Bus Voltage   : %.3f V" % ina.get_bus_voltage_V())
#        print ("Current       : %.f mA" % ina.get_current_mA())
#        print ("Power         : %.f mW" % ina.get_power_mW())
#        print (" ")

#if __name__ == "__main__":
#    main()
