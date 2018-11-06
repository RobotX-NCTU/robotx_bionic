#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry


class MocapLocalizationNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # base tag id~system_number
        self.system_number = rospy.get_param("~system_number", 1)
        #print self.system_number

        # base tag id      
        #self.base_tag_id = [505, 504, 507]
        self.base_tag_id_group = [[504, 505, 506, 507],[508, 509, 510, 511]]
        self.base_tag_id = self.base_tag_id_group[self.system_number-1]
        # vehicle tag id    
        self.vehicle_tag_id = [501, 502, 503]

        # base tag groundtruth point
        self.base_tag_point = np.array([[0, 0, 0], [20, 0, 0], [0, 20, 0], [20, 20, 0]], dtype='f')
        # base tag detection point     
        #self.obser_tag_point = np.zeros((3, 3), dtype='f')
        #self.test_tag_point = np.zeros((3, 4), dtype='f')

        # vehicle tag detection point
        #self.vehicle_tag_point_pair = np.zeros((3, 4), dtype='f')

        # legal or illegal localization
        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTagDetections, queue_size=1)
        # Publishers
        self.pub_vehicle_pose_pair = rospy.Publisher("~vehicle_pose_pair", PoseArray, queue_size=1)
        self.pub_odom = rospy.Publisher('~tag_localization_odometry', Odometry, queue_size = 20)

    def processTagDetections(self,tag_detections_msg):
        ## print "-----------------------------------------------"
        # assign base tag coordination
        self.base_tag_point = np.array([[0, 0, 0], [20, 0, 0], [0, 20, 0], [20, 20, 0]], dtype='f') 
        self.vehicle_tag_point_pair = np.zeros((3, 4), dtype='f')
        self.obser_tag_point = np.zeros((4, 3), dtype='f')
        self.test_tag_point = np.zeros((4, 4), dtype='f')

        for tag_detection in tag_detections_msg.detections:
            print tag_detection.id[0]
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
        print 'system: ', self.system_number
        print 'base tag count:',self.base_tag_detect_count 
        print 'vehicle tag count:',self.vehicle_tag_detect_count                    
        if(self.base_tag_detect_count < 3 or self.vehicle_tag_detect_count == 0):
            self.base_tag_detect_count = 0
            self.vehicle_tag_detect_count = 0
            print 'non enough tags detectecd'
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

        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

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

        print "vehicle tag detection"
        print self.vehicle_tag_point_pair.transpose()
        print "vehicle tag detection after transformation"
        print np.dot(T,self.vehicle_tag_point_pair.transpose())

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

    def cb_odom(self, point):
        odom_msg = Odometry() 
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position = point
        print "odem", point
        self.pub_odom.publish(odom_msg)

    def onShutdown(self):
        rospy.loginfo("[MocapLocalizationNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('mocap_localization_node',anonymous=False)
    mocap_localization_node = MocapLocalizationNode()
    rospy.on_shutdown(mocap_localization_node.onShutdown)
    rospy.spin()
