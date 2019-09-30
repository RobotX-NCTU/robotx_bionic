#!/usr/bin/env python
import rospy
import time
import datetime
import xlwt
from robotx_bionic_msgs.msg import TemperatureCurrent
from os.path import expanduser

class DataToXlsNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		starting_time = rospy.get_param("~starting_time", "2018-10-03-10-42-00")

		# subscriber
		self.temp_press_sub = rospy.Subscriber('/temp_current_8A_node/temp_current', TemperatureCurrent, self.cb_temp_press, queue_size = 10)

        # initial xls file
		self.book = xlwt.Workbook(encoding="utf-8")
		self.sheet1 = self.book.add_sheet("Sheet 1")
		self.sheet1.write(0, 0, "Original Time")
		self.sheet1.write(0, 1, "True Time")
		self.sheet1.write(0, 2, "Power")
		self.sheet1.write(0, 3, "Current")
		self.sheet1.write(0, 4, "Voltage")
		self.sheet1.write(0, 5, "T_CPU")
		self.home = expanduser("~")

        # get real time shift
		self.index = 0
		fmt = "%Y-%m-%d-%H-%M-%S"
		self.time_mapping = datetime.datetime.strptime(starting_time, fmt)
		self.get_initial_time = False
		self.initial_time = []

	def cb_temp_press(self, temp_msg):
		# record initial time
		if not self.get_initial_time:
			self.initial_time = datetime.datetime.fromtimestamp(temp_msg.header.stamp.secs+temp_msg.header.stamp.nsecs*1e-9)
			self.get_initial_time = True

        # adding real time shift
		original_time = datetime.datetime.fromtimestamp(temp_msg.header.stamp.secs+temp_msg.header.stamp.nsecs*1e-9)
		real_time = self.time_mapping + (original_time - self.initial_time) 

        # writing sensor data
		self.index += 1
		self.sheet1.write(self.index, 0, str(original_time))
		self.sheet1.write(self.index, 1, str(real_time))
		self.sheet1.write(self.index, 2, str(temp_msg.power))
		self.sheet1.write(self.index, 3, str(temp_msg.current))
		self.sheet1.write(self.index, 4, str(temp_msg.voltage))
		self.sheet1.write(self.index, 5, str(temp_msg.temperature_cpu))

	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown " %(self.node_name))
		# save the xls file
		self.book.save(self.home + "/temp_press.xls")
		print "save data to ", (self.home+"/temp_press.xls")

if __name__ == '__main__':
	rospy.init_node("data_to_xls_node", anonymous = False)
	data_to_xls_node = DataToXlsNode()
	rospy.on_shutdown(data_to_xls_node.onShutdown)
	rospy.spin()
