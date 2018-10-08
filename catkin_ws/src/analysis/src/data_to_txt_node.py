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
import time
import datetime
import xlwt
from robotx_bionic_msgs.msg import TemperaturePressure

class DataToXlsNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.temp_press_sub = rospy.Subscriber('/bamboobota/temp_press_serial_node/temp_press', TemperaturePressure, self.cb_temp_press, queue_size = 10)



		self.book = xlwt.Workbook(encoding="utf-8")
		self.sheet1 = self.book.add_sheet("Sheet 1")
		self.sheet1.write(0, 0, "Original Time")
		self.sheet1.write(0, 1, "True Time")
		self.sheet1.write(0, 2, "Temperature")
		self.sheet1.write(0, 3, "Pressure")
		self.index = 0
		fmt = '%Y-%m-%d %H:%M:%S'
		self.time_mapping = datetime.datetime.strptime('2018-10-03 10:42:00', fmt)
		self.time_mapping = datetime.datetime(2018, 10, 03, 10, 42, 00)
		self.get_initial_time = False
		self.initial_time = []





	def cb_temp_press(self, temp_press_msg):
		if not self.get_initial_time:

			self.initial_time = datetime.datetime.fromtimestamp(temp_press_msg.header.stamp.secs+temp_press_msg.header.stamp.nsecs*1e-9)
			self.get_initial_time = True
		original_date = datetime.datetime.fromtimestamp(temp_press_msg.header.stamp.secs+temp_press_msg.header.stamp.nsecs*1e-9)
		real_date = self.time_mapping + (original_date - self.initial_time) 
		#real_date = datetime.datetime.fromtimestamp(mapping_secs)
		print real_date, temp_press_msg.temperature, temp_press_msg.pressure
		self.index += 1
		self.sheet1.write(self.index, 0, str(original_date))
		self.sheet1.write(self.index, 1, str(real_date))
		self.sheet1.write(self.index, 2, str(temp_press_msg.temperature))
		self.sheet1.write(self.index, 3, str(temp_press_msg.pressure))



	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown " %(self.node_name))
		self.book.save("trial.xls")

if __name__ == '__main__':
	rospy.init_node("data_to_xls_node", anonymous = False)
	data_to_xls_node = DataToXlsNode()
	rospy.on_shutdown(data_to_xls_node.onShutdown)
	rospy.spin()