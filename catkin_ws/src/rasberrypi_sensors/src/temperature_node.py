#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from robotx_bionic_msgs.msg import Temperature

import time



class Temperature_node(object):
        def __init__(self):
                self.node_name = rospy.get_name()
                rospy.loginfo("[%s] Initializing " %(self.node_name))
                self.temp_pub = rospy.Publisher('~temp', Temperature, queue_size = 10)

        def cb(self, no_use):
                temp_cpu = self.get_cpu_temp()
                print datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                print "temperature on cpu: ", str(temp_cpu)
                print "---------------------"
                temp_msg = Temperature()
                temp_msg.header.stamp = rospy.Time.now()
                temp_msg.temperature_cpu = str(temp_cpu)
                self.temp_pub.publish(temp_msg)

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
        rospy.init_node("temperature_node", anonymous = False)
        temperature_node = Temperature_node()
        rospy.on_shutdown(temperature_node.onShutdown)
        rospy.Timer(rospy.Duration(2), temperature_node.cb)  # 15
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
