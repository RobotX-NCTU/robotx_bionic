#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix, rotation_matrix
from visualization_msgs.msg import Marker, MarkerArray


class MocapLocalizationNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # visualization
        self.verbose = rospy.get_param("~verbose", False)
        self.simulation = rospy.get_param("~simulation", True)

        # base tag id~system_number
        self.system_number = rospy.get_param("~system_number", 1)

        # base tag id
        if self.simulation:      
            self.base_tag_id_group = [[504, 505, 506, 507],[508, 509, 510, 511]]
            self.base_tag_id = self.base_tag_id_group[self.system_number-1]
        else:
            self.base_tag_id_group = [[505, 507, 504, 506],[508, 509, 510, 511]]
            self.base_tag_id = self.base_tag_id_group[self.system_number-1]
        # vehicle tag id    
        self.vehicle_tag_id = [501, 502, 503, 512] # left, back, right, front

        # base tag groundtruth point
        #if self.simulation:
        #    self.base_tag_point = np.array([[0, 0, 0.7], [20, 0, 0.7], [0, 20, 0.7], [20, 20, 0.7]], dtype='f')
        #else:
        #    self.base_tag_point = np.array([[0, 0, 0.7], [10, 0, 0.7], [0, 10, 0.7], [10, 10, 0.7]], dtype='f')

        # for fix mapping matric
        self.get_mapping_matrix = False
        self.T = []

        # transformation for real data
        angles = [0, 0, -0.68 * math.pi ]
        trans = [-7, 7, 0]
        #print angles, trans
        self.M0 = compose_matrix(scale=None, shear=None, angles=angles, translate=trans, perspective=None)
        #print self.M0

        # shift bwtween tags and center of wamv
        #self.shift_center = np.array([[0.45, -0.5, 0, 0], [0.95, 0, 0, 0], [0.45, 0.5, 0, 0], [-1.1, 0.0, -0.9, 0]], dtype='f')
        self.shift_center = np.array([[0.5, -0.5, 0, 0], [0.95, 0, 0, 0], [0.5, 0.5, 0, 0], [1.1, 0.0, -0.9, 0]], dtype='f')
        self.shift_phi = np.array([math.pi*0.5, math.pi, math.pi, 0], dtype='f')
        self.vehicle_theta_pre = np.zeros((4, 1), dtype='f')

        # legal or illegal localization
        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        # previous position of vehicle
        self.pre_vehicle_loalization = Point()
        self.count = 0

        # apriltag cube marker
        if self.simulation == True:
            self.apriltag_cube_point = np.array([[0, 0, 0.7], [20, 0, 0.7], [0, 20, 0.7], [20, 20, 0.7]], dtype='f')
        else:
        	# for bamboolake 20181116 
            #self.apriltag_cube_point = np.array([[0, 0, 0.7], [13.14, 0, 0.7], [750, 12.87, 0.7], [16.72, 14.62, 0.7]], dtype='f')
            #self.apriltag_cube_point = np.array([[0, 0, 0.7], [12, 1, 0.7], [-0.75, 12.17, 0.7], [14.8, 13.02, 0.7]], dtype='f')
            #self.apriltag_cube_point = np.array([[-0.75, -0.75, 0.7], [10, -1, 0.7], [-1.25, 11.17, 0.7], [14.8, 11.52, 0.7]], dtype='f')
            self.apriltag_cube_point = np.array([[14, 10, 0.7], [23, 10, 0.7], [13.5, 23.5, 0.7], [29, 22, 0.7]], dtype='f')
            #self.apriltag_cube_point = np.array([[19, 7, 0.7], [28, 5.5, 0.7], [17.5, 19.5, 0.7], [34, 18, 0.7]], dtype='f')
            
            #self.obs_sphere_point = []
            self.obs_sphere_point = np.array([[-2.5, -11, 0.7], [-9, -24, 0.7], [-1.5, -21, 0.7]], dtype='f')
            #self.obs_sphere_point = np.array([[-9, -15, 0.7], [-13, -26.5, 0.7], [-3.5, -24, 0.7]], dtype='f')
            
            print self.apriltag_cube_point
            for i in range(4):
    
                #print i
                origin_point = [self.apriltag_cube_point[i][0], self.apriltag_cube_point[i][1], self.apriltag_cube_point[i][2], 1]
                #origin_point = compose_matrix(scale=None, shear=None, angles=angles, translate=trans, perspective=None)
                #print origin_point
                #print self.M0
                origin_after = self.M0.dot(origin_point)
                #print origin_after
                self.apriltag_cube_point[i][0] = origin_after[0]
                self.apriltag_cube_point[i][1] = origin_after[1]
                self.apriltag_cube_point[i][2] = origin_after[2]
            print self.apriltag_cube_point

        self.marker_array = MarkerArray()

        self.marker = Marker()
        self.marker.header.frame_id = 'odom'
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.r = 1
        self.marker.color.g = 1
        self.marker.color.b = 1
        self.marker.color.a = 0.5

        # apriltag localization path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTagDetections, queue_size=10)
        
        # Publishers
        self.pub_vehicle_pose = rospy.Publisher("~als_posestamped", PoseStamped, queue_size=10)
        self.pub_visual_apriltag_wp = rospy.Publisher("~apriltag_cube_waypoint", MarkerArray, queue_size=20)
        #self.pub_obstacle = rospy.Publisher("~obstacle", MarkerArray, queue_size=20)
        # self.pub_odom = rospy.Publisher('~tag_localization_odometry', Odometry, queue_size = 20)
        # self.pub_path = rospy.Publisher('~tag_localization_path', Path, queue_size = 20)

    def processTagDetections(self,tag_detections_msg):
        # assign base tag coordination
        # for gazebo
        #self.base_tag_point = np.array([[0, 0, 0.7], [20, 0, 0.7], [0, 20, 0.7], [20, 20, 0.7]], dtype='f')
        self.base_tag_point = self.apriltag_cube_point
        # for bamboolake 20181117
        #self.base_tag_point = np.array([[0, 0, 0.7], [10.59, 0, 0.7], [0, 12.55, 0.7], [16.45, 13.934, 0.7]], dtype='f')

        for p in self.base_tag_point:
            self.marker.pose.position = Vector3(p[0], p[1], p[2])
            self.marker.id += 1
            self.marker.type = Marker.CUBE
            self.marker.color.r = 1
            self.marker.color.g = 1
            self.marker.color.b = 1
            self.marker_array.markers.append(self.marker)
            self.pub_visual_apriltag_wp.publish(self.marker_array)

        for p in self.obs_sphere_point:
            self.marker.pose.position = Vector3(p[0], p[1], p[2])
            self.marker.id += 1
            self.marker.type = Marker.SPHERE
            self.marker.color.r = 0
            self.marker.color.g = 1
            self.marker.color.b = 0
            self.marker_array.markers.append(self.marker)
            self.pub_visual_apriltag_wp.publish(self.marker_array)

        self.vehicle_tag_point_pair = np.zeros((4, 4), dtype='f')
        self.vehicle_theta = np.zeros((4, 1), dtype='f')
        self.vehicle_check = np.zeros((4, 1), dtype='f')
        self.obser_tag_point = np.zeros((4, 3), dtype='f')
        self.test_tag_point = np.zeros((4, 4), dtype='f')

        # assign header
        self.header = tag_detections_msg.header

        for tag_detection in tag_detections_msg.detections:
            #if(self.verbose == False): print tag_detection.id[0]
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
                if tag_detection.id[0] == tag_id:
                    a = tag_detection.pose.pose.pose.orientation
                    n = euler_from_quaternion([a.x, a.y, a.z, a.w]) 
                    #if(self.verbose == False): print '============================='
                    #if(self.verbose == False): print tag_id,'yaw:', n[1]/3.14159*360, n[1]
                    #if(self.verbose == False): print '============================='
                    if(self.verbose): print 'tag_id: ', tag_id, n 

                    # remove unreasonable detection
                    #if abs(n[0]) < 3 or abs(n[2]) < 0.00001:
                    #if abs(n[0]) < 3.05 :
                    #if abs(n[0]) < 3.05 :
                        #break
                        #return
                    #print tag_id, n

                    self.vehicle_theta[index, 0] = n[1]
                    self.vehicle_check[index, 0] = abs(n[1])

                    self.vehicle_tag_point_pair[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.vehicle_tag_point_pair[index, 1] = tag_detection.pose.pose.pose.position.x               
                    self.vehicle_tag_point_pair[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.vehicle_tag_point_pair[index, 3] = 1

                    self.vehicle_tag_detect_count += 1 

        # check whether two pitch of adjacent tag detections are half of pi
        if self.vehicle_tag_detect_count == 2:
            theta_sum = self.vehicle_check.sum(axis=0)
            print "sum of theta: ", theta_sum
            if abs(theta_sum - math.pi * 0.5) > 0.2:
                print "illegal detection"
                self.base_tag_detect_count = 0
                self.vehicle_tag_detect_count = 0
                return
        # check whether pitch variance of same tag detection is too sharp
        for i in range(4):
            if((self.vehicle_theta[i, 0] * self.vehicle_theta_pre[i, 0]) < -0.4):
                print "too large varing yaw"
                self.base_tag_detect_count = 0
                self.vehicle_tag_detect_count = 0
                for j in range(4):
                    a = self.vehicle_theta[i, 0]
                    self.vehicle_theta_pre[i, 0] = a
                return
        for j in range(4):
            a = self.vehicle_theta[i, 0]
            self.vehicle_theta_pre[i, 0] = a
        if(self.verbose): print self.vehicle_theta
        if(self.verbose): print self.shift_phi

        # check enough tags detected
        if(self.verbose): print 'system: ', self.system_number
        print 'base tag count:',self.base_tag_detect_count 
        print 'vehicle tag count:',self.vehicle_tag_detect_count
        if self.get_mapping_matrix == False:                    
            if(self.base_tag_detect_count < 3 or self.vehicle_tag_detect_count == 0):
                self.base_tag_detect_count = 0
                self.vehicle_tag_detect_count = 0
                if(self.verbose): print 'non enough tags detectecd'
                rospy.loginfo("non enough tags detectecd")
                return
        else:                    
            if(self.vehicle_tag_detect_count == 0):
                self.base_tag_detect_count = 0
                self.vehicle_tag_detect_count = 0
                if(self.verbose): print 'non enough tags detectecd'
                rospy.loginfo("non enough tags detectecd")
                return                

        # shift compensation
        for i in range(4):
            if self.vehicle_tag_point_pair[i,2] != 0:
                dx = (-1 * abs(self.shift_center[i,0]) * math.cos(math.pi+self.vehicle_theta[i,0]))
                dy = (1 * abs(self.shift_center[i,0]) * math.sin(math.pi+self.vehicle_theta[i,0]))
                if(self.verbose): print dx
                if(self.verbose): print dy
                self.vehicle_tag_point_pair[i, 0] += dx 
                self.vehicle_tag_point_pair[i, 1] -= dy        

        # remove zero row if only three base apriltag detected
        if(self.base_tag_detect_count == 3):
            zero_row = np.where(~self.obser_tag_point.any(axis=1))[0][0]
            self.base_tag_point = np.delete(self.base_tag_point, zero_row, axis=0)
            self.obser_tag_point = self.obser_tag_point[~(self.obser_tag_point==0).all(1)]

        #if True:
        if self.get_mapping_matrix == False:
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
            self.T = T
            self.get_mapping_matrix =True


        #if(self.verbose): print "vehicle tag detection"
        #if(self.verbose): print self.vehicle_tag_point_pair.transpose()
        #if(self.verbose): print "vehicle tag detection after transformation"
        #if(self.verbose): print np.dot(self.T,self.vehicle_tag_point_pair.transpose())
        #if(self.verbose): print "vehicle tag detection"
        #print "vehicle tag detection"
        #print self.vehicle_tag_point_pair.transpose()
        #print "vehicle tag detection after transformation"
        #print np.dot(self.T,self.vehicle_tag_point_pair.transpose())

        vehicle_loalization = np.dot(self.T,self.vehicle_tag_point_pair.transpose())
        shift = self.shift_center.transpose()
        #for i in range(3):
        #    if vehicle_loalization[:,i][3] != 0:
        #        for j in range(4):
        #            vehicle_loalization[:,i][j] += shift[:,i][j]
        #            vehicle_loalization[:,i][j] = vehicle_loalization[:,i][j] 
        if(self.verbose): print "after shift"
        if(self.verbose): print vehicle_loalization
        vehicle_loalization = vehicle_loalization.sum(axis=1)/self.vehicle_tag_detect_count
        #vehicle_loalization[2] = 0
        # "average", vehicle_loalization

        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        vehicle_pose = Pose()
        vehicle_pose.position.x = vehicle_loalization[0] 
        vehicle_pose.position.y = vehicle_loalization[1] 
        vehicle_pose.position.z = vehicle_loalization[2]
        print vehicle_pose.position.x, vehicle_pose.position.y, vehicle_pose.position.z
        #print "average", vehicle_loalization
        # publish odometry
        #self.cb_odom(vehicle_pose.position)

        # publish path
        # self.cb_path(vehicle_pose)
        #print self.T
        # publish posestamped
        self.cb_pose_stam(vehicle_pose)
        self.count += 1
        #print 'total: ', self.count

        #self.pre_vehicle_loalization = vehicle_pose.position

    def cb_pose_stam(self, p):
        pose_stamped_msg = PoseStamped()


        pose_stamped_msg.header = self.header
        pose_stamped_msg.header.frame_id = "odom"

        pose_stamped_msg.pose = p

        self.pub_vehicle_pose.publish(pose_stamped_msg)
        if(self.verbose): print 'pose'
        if(self.verbose): print pose_stamped_msg.pose
        if(self.verbose): print '------------'

    #def cb_path(self, p):
    #    pose_stamped = PoseStamped()

        #self.path_msg.header.stamp = rospy.Time.now()
        #self.path_msg.header.seq = pose_stamped.header.seq
        #pose_stamped.header.stamp = self.path_msg.header.stamp
    #    pose_stamped.header = self.header
    #    pose_stamped.header.frame_id = "odom"

    #    pose_stamped.pose = p

    #    self.path_msg.poses.append(pose_stamped)
    #    #self.pub_path.publish(self.path_msg)
    #    print 'poses------------'
    #    print self.path_msg.poses
    #    print '------------'


    #def cb_odom(self, point):
    #    odom_msg = Odometry() 

    #    odom_msg.header = self.header
    #    # reassign frame_id
    #    odom_msg.header.frame_id = "odom"
    #    #odom_msg.header.stamp = rospy.Time.now()

    #    odom_msg.pose.pose.position = point
    #    #odom_msg.pose.pose.orientation = self.get_orientation(point)
    #    print "odem", odom_msg.pose.pose
    #    #self.pub_odom.publish(odom_msg)

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
