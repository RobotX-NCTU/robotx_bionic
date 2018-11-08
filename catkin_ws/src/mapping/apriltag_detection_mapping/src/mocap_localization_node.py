#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler


class MocapLocalizationNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose=False

        # base tag id~system_number
        self.system_number = rospy.get_param("~system_number", 1)

        # base tag id      
        self.base_tag_id_group = [[504, 505, 506, 507],[508, 509, 510, 511]]
        self.base_tag_id = self.base_tag_id_group[self.system_number-1]
        # vehicle tag id    
        self.vehicle_tag_id = [501, 502, 503]

        # base tag groundtruth point
        self.base_tag_point = np.array([[0, 0, 0], [20, 0, 0], [0, 20, 0], [20, 20, 0]], dtype='f')

        # shift between gps and apriltag localization
        self.shift_x = 5 - 0.25
        self.shift_y = -5 + 0.25
        # shift bwtween tag and center of wamv
        self.shift_center = np.array([[0.45, -0.5, 0, 0], [0.95, 0, 0, 0], [0.45, 0.5, 0, 0]], dtype='f')

        # legal or illegal localization
        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        # previous position of vehicle
        self.pre_vehicle_loalization = Point()

        # apriltag localization path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTagDetections, queue_size=1)
        # Publishers
        #self.pub_vehicle_pose_pair = rospy.Publisher("~vehicle_pose_pair", PoseArray, queue_size=1)
        #.pub_odom = rospy.Publisher('~tag_localization_odometry', Odometry, queue_size = 20)
        self.pub_path = rospy.Publisher('~tag_localization_path', Path, queue_size = 20)

    def processTagDetections(self,tag_detections_msg):
        ## print "-----------------------------------------------"
        # assign base tag coordination
        self.base_tag_point = np.array([[0, 0, 0.7], [20, 0, 0.7], [0, 20, 0.7], [20, 20, 0.7]], dtype='f') 
        self.vehicle_tag_point_pair = np.zeros((3, 4), dtype='f')
        self.obser_tag_point = np.zeros((4, 3), dtype='f')
        self.test_tag_point = np.zeros((4, 4), dtype='f')

        for tag_detection in tag_detections_msg.detections:
            if(self.verbose): print tag_detection.id[0]
        	# extract base tag detection
            for index, tag_id in enumerate(self.base_tag_id):
                if tag_detection.id[0] == tag_id:
                    self.obser_tag_point[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.obser_tag_point[index, 1] = tag_detection.pose.pose.pose.position.x                   
                    self.obser_tag_point[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.test_tag_point[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.test_tag_point[index, 1] =  tag_detection.pose.pose.pose.position.x                  
                    self.test_tag_point[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.test_tag_point[index, 3] = 1
                    self.base_tag_detect_count += 1
            # extract vehicle tag detection
            for index, tag_id in enumerate(self.vehicle_tag_id):
                #print tag_detection.id[0], tag_id
                if tag_detection.id[0] == tag_id:
                    self.vehicle_tag_point_pair[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.vehicle_tag_point_pair[index, 1] = tag_detection.pose.pose.pose.position.x               
                    self.vehicle_tag_point_pair[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.vehicle_tag_point_pair[index, 3] = 1 
                    self.vehicle_tag_detect_count += 1 

        # check enough tags detected
        if(self.verbose): print 'system: ', self.system_number
        if(self.verbose): print 'base tag count:',self.base_tag_detect_count 
        if(self.verbose): print 'vehicle tag count:',self.vehicle_tag_detect_count                    
        if(self.base_tag_detect_count < 3 or self.vehicle_tag_detect_count == 0):
            self.base_tag_detect_count = 0
            self.vehicle_tag_detect_count = 0
            if(self.verbose): print 'non enough tags detectecd'
            rospy.loginfo("non enough tags detectecd")
            return

        # remove zero row if only three base apriltag detected
        if(self.base_tag_detect_count == 3):
            zero_row = np.where(~self.obser_tag_point.any(axis=1))[0][0]
            #np.delete(self.base_tag_point, 1, 0)
            self.base_tag_point = np.delete(self.base_tag_point, zero_row, axis=0)
            #print "row", zero_row
            self.obser_tag_point = self.obser_tag_point[~(self.obser_tag_point==0).all(1)]

        length = self.obser_tag_point.shape[0]
        p_ct = self.base_tag_point.sum(axis=0) / length
        p_cm = self.obser_tag_point.sum(axis=0) / length

        self.obser_tag_point = self.obser_tag_point - p_cm
        self.base_tag_point = self.base_tag_point - p_cm

        Mtd = self.base_tag_point[0]
        Mmd = self.obser_tag_point[0]
        for i in range(length-1):
            Mtd = np.vstack((Mtd, self.base_tag_point[i+1]))
            Mmd = np.vstack((Mmd, self.obser_tag_point[i+1]))
        Mtd = Mtd.transpose()
        Mmd = Mmd.transpose()

        #p_ct = (self.base_tag_point[0] + self.base_tag_point[1] + self.base_tag_point[2])/3
        #p_cm = (self.obser_tag_point[0] + self.obser_tag_point[1] + self.obser_tag_point[2])/3

        #for i in range(3):
        #    self.obser_tag_point[i] = self.obser_tag_point[i] - p_cm
        #    self.base_tag_point[i] = self.base_tag_point[i] - p_ct

        #Mtd = np.vstack((self.base_tag_point[0], self.base_tag_point[1], self.base_tag_point[2])).transpose()
        #Mmd = np.vstack((self.obser_tag_point[0], self.obser_tag_point[1], self.obser_tag_point[2])).transpose()

        H = np.dot(Mmd, Mtd.transpose())
        [U, D, V] = np.linalg.svd(H,full_matrices=1)
        R = np.dot(V,U.transpose())
        t = np.matrix(p_ct - np.dot(R,p_cm))

        temp = np.hstack((R,t.transpose()))
        zero = np.array([[0,0,0,1]])
        T = np.vstack((temp,zero))

        #print "tag detection"
        #print self.test_tag_point.transpose()
        #print "tag detection after transformation"
        #print np.dot(T,self.test_tag_point.transpose())

        if(self.verbose): print "vehicle tag detection"
        if(self.verbose): print self.vehicle_tag_point_pair.transpose()
        if(self.verbose): print "vehicle tag detection after transformation"
        if(self.verbose): print np.dot(T,self.vehicle_tag_point_pair.transpose())


        vehicle_loalization = np.dot(T,self.vehicle_tag_point_pair.transpose())
        shift = self.shift_center.transpose()
        for i in range(3):
            if vehicle_loalization[:,i][3] != 0:
                for j in range(4):
                    vehicle_loalization[:,i][j] += shift[:,i][j]
                    #print vehicle_loalization[:,i][0]
        if(self.verbose): print "after shift"
        if(self.verbose): print vehicle_loalization
        vehicle_loalization = vehicle_loalization.sum(axis=1)/self.vehicle_tag_detect_count
        vehicle_loalization[2] = 0
        if(self.verbose): print vehicle_loalization


        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        vehicle_pose = Pose()
        vehicle_pose.position.x = vehicle_loalization[0] + self.shift_x
        vehicle_pose.position.y = vehicle_loalization[1] + self.shift_y
        vehicle_pose.position.z = vehicle_loalization[2]
        #self.cb_odom(vehicle_pose.position)
        self.cb_path(vehicle_pose)
        self.pre_vehicle_loalization = vehicle_pose.position

        #vehicle_point_pair = np.zeros((2, 4), dtype='f')
        #vehicle_point_pair = np.dot(T,self.vehicle_tag_point_pair.transpose())
        #vehicle_pose_pair_msg = PoseArray()
        #for i in range(2):
        #    vehicle_pose = Pose()
        #    vehicle_pose.position.x = vehicle_point_pair[0,i]
        #    vehicle_pose.position.y = vehicle_point_pair[1,i]           
        #    vehicle_pose.position.z = vehicle_point_pair[2,i]
        #    vehicle_pose_pair_msg.poses.append(vehicle_pose)
        #self.pub_vehicle_pose_pair.publish(vehicle_pose_pair_msg)

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


    def cb_odom(self, point):
        odom_msg = Odometry() 
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position = point
        #odom_msg.pose.pose.orientation = self.get_orientation(point)
        print "odem", odom_msg.pose.pose
        self.pub_odom.publish(odom_msg)

    def get_orientation(self, point):
        dx = point.x - self.pre_vehicle_loalization.x
        dy = point.y - self.pre_vehicle_loalization.y
        quat_msg = Quaternion()
        self.quat = []
        if(dx ==0):
            self.quat = quaternion_from_euler (0, 0, math.pi*0.5)
        else:
            theta = math.atan2(dy, dx)
            if(theta < 0):
                theta += math.pi * 2
            self.quat = quaternion_from_euler (0, 0, theta)
        print dx, dy, self.quat
        quat_msg.x = self.quat[0]
        quat_msg.y = self.quat[1]
        quat_msg.z = self.quat[2]
        quat_msg.w = self.quat[3]
        return quat_msg


    def onShutdown(self):
        rospy.loginfo("[MocapLocalizationNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('mocap_localization_node',anonymous=False)
    mocap_localization_node = MocapLocalizationNode()
    rospy.on_shutdown(mocap_localization_node.onShutdown)
    rospy.spin()
