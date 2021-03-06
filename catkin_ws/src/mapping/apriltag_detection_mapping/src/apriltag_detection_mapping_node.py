#!/usr/bin/env python
# Transfer apriltag detection to gazabo model state of WAM-V
# Editor: Brian Chuang
# Last update: 10/09/2018
#'''
#	Update: initial node (10/06/2018)
#'''

import rospy
import math
import tf
import numpy as np
from gazebo_msgs.msg import ModelState
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped


class ApriltagDetectionMappingNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing" %(self.node_name))

		self.verbose=False

		# Subscriber
		self.sub_tag_detect = rospy.Subscriber('/watchtower_left/wamv/tag_detections', AprilTagDetectionArray, self.cb_tag_detect, queue_size = 10)

		# Publisher
		self.pub_wamv_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)

		# Parameters
		self.wamv_model_name = rospy.get_param("~wamv_model_name", "wamv")
		self.tag_id = rospy.get_param("~tag_id", 503)
		self.tf_translation = [0, 0, 0]
		self.tf_rotation = [math.pi, 0, 0]

		# apriltag localization path
		self.path_msg = Path()
		self.path_msg.header.frame_id = "odom"

		self.pub_path = rospy.Publisher('~tag_localization_path', Path, queue_size = 20)

	def cb_tag_detect(self, detections_msg):

		for i in range(len(detections_msg.detections)):
			print "id ", self.tag_id
			if(detections_msg.detections[i].id[0] == self.tag_id):
				if(self.verbose):
					print "id ", self.tag_id, " detected and transfered to WAM-V state"
				self.detection_to_state(detections_msg.detections[i])

	def detection_to_state(self, detection):
		wamv_state_msg = ModelState()
		wamv_state_msg.model_name = self.wamv_model_name
		wamv_state_msg.pose = detection.pose.pose.pose
		#print  "original pose: ", wamv_state_msg.pose 

		trans_mat = tf.transformations.compose_matrix(None, None, self.tf_rotation, self.tf_translation, None)
		#print "transformation matrix: ", trans_mat

		pos = wamv_state_msg.pose.position
		quan = wamv_state_msg.pose.orientation
		original_position = [pos.z, -1 * pos.x, 0.1]
		original_rotation = tf.transformations.euler_from_quaternion([quan.x, quan.y, quan.z, quan.w])
		original_rotation = [original_rotation[0], original_rotation[2], -1 * original_rotation[1]]
		original_mat = tf.transformations.compose_matrix(None, None, original_rotation, original_position, None)
		#print "original pose matrix: ", original_mat
		if(self.verbose): print "original original_rotation: ", original_position 
		transformed_mat = np.dot(original_mat, trans_mat)
		#print  "transformed pose matrix: ", transformed_mat

		[ scale, shear, angles, trans, persp] = tf.transformations.decompose_matrix(transformed_mat)
		angles = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])

		wamv_state_msg.pose.position.x = trans[0]
		wamv_state_msg.pose.position.y = trans[1]
		wamv_state_msg.pose.position.z = trans[2]

		wamv_state_msg.pose.orientation.x = angles[0]
		wamv_state_msg.pose.orientation.y = angles[1]
		wamv_state_msg.pose.orientation.z = angles[2]
		wamv_state_msg.pose.orientation.w = angles[3]
		self.cb_path(wamv_state_msg.pose)
		self.pub_wamv_state.publish(wamv_state_msg)

	def cb_path(self, p):
		pose_stamped = PoseStamped()
		pose_stamped.header.frame_id = "odom"
		pose_stamped.pose = p
		self.path_msg.header.stamp = rospy.Time.now()
		self.path_msg.header.seq = pose_stamped.header.seq
		pose_stamped.header.stamp = self.path_msg.header.stamp
		self.path_msg.poses.append(pose_stamped)
		self.pub_path.publish(self.path_msg)
		print 'poses------------'
		print self.path_msg.poses
		print '------------'


	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
	rospy.init_node("apriltag_detection_mappping_node", anonymous = False)
	apriltag_detection_mappping_node = ApriltagDetectionMappingNode()
	rospy.on_shutdown(apriltag_detection_mappping_node.onShutdown)
	rospy.spin()
