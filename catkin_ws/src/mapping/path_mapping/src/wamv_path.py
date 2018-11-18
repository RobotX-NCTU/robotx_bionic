#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
import os
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int32


class WAMVPath(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose = False

        self.path_group = 5
 
        self.channel = 1

        self.write_ground_truth = dict()
        self.write_gps = dict()
        self.write_als = dict()
        save_folder = (os.getenv("HOME") + '/wamv_path/')
        if not (os.path.isdir(save_folder)):
            print "create", save_folder, "for saving wamv path"
            os.makedirs(save_folder)
        print "save path results to", save_folder

        for i in range(self.path_group):
            self.write_ground_truth[i] = open( (save_folder) + "/gt_path" + str(i+1) + ".txt", "a")
            self.write_gps[i] = open( (save_folder) + "/gps_path" + str(i+1) + ".txt", "a")
            self.write_als[i] = open( (save_folder) + "/als_path" + str(i+1) + ".txt", "a")

        # mapping all path
        self.is_ground_truth_first = True
        self.is_als_first = True
        self.is_gps_first = True
        self.ground_truth_first = Pose()
        self.als_first = Pose()
        self.pgs_first = Pose()

        # apriltag localization path
        self.grame_id = "odom"
        self.ground_truth_path_msg = dict()
        self.gps_path_msg = dict()
        self.als_path_msg = dict()

        for i in range(self.path_group):
            self.ground_truth_path_msg[i] = Path()
            self.ground_truth_path_msg[i].header.frame_id = self.grame_id
            self.als_path_msg[i] = Path()
            self.als_path_msg[i].header.frame_id = self.grame_id
            self.gps_path_msg[i] = Path()
            self.gps_path_msg[i].header.frame_id = self.grame_id                      

        """
        self.ground_truth_path_msg = Path()
        self.ground_truth_path_msg.header.frame_id = self.grame_id
        self.gps_path_msg = Path()
        self.gps_path_msg.header.frame_id = self.grame_id
        self.als_path_msg = Path()
        self.als_path_msg.header.frame_id = self.grame_id

        self.ground_truth_path_msg2 = Path()
        self.ground_truth_path_msg2.header.frame_id = self.grame_id
        self.gps_path_msg2 = Path()
        self.gps_path_msg2.header.frame_id = self.grame_id
        self.als_path_msg2 = Path()
        self.als_path_msg2.header.frame_id = self.grame_id

        self.ground_truth_path_msg3 = Path()
        self.ground_truth_path_msg3.header.frame_id = self.grame_id
        self.gps_path_msg3 = Path()
        self.gps_path_msg3.header.frame_id = self.grame_id
        self.als_path_msg3 = Path()
        self.als_path_msg3.header.frame_id = self.grame_id

        self.ground_truth_path_msg4 = Path()
        self.ground_truth_path_msg4.header.frame_id = self.grame_id
        self.gps_path_msg4 = Path()
        self.gps_path_msg4.header.frame_id = self.grame_id
        self.als_path_msg4 = Path()
        self.als_path_msg4.header.frame_id = self.grame_id

        self.ground_truth_path_msg5 = Path()
        self.ground_truth_path_msg5.header.frame_id = self.grame_id
        self.gps_path_msg5 = Path()
        self.gps_path_msg5.header.frame_id = self.grame_id
        self.als_path_msg5 = Path()
        self.als_path_msg5.header.frame_id = self.grame_id
        """

        self.pub_ground_truth_topic = '~ground_truth_path'
        self.pub_als_topic = '~als_path'
        self.pub_gps_topic = '~gps_path'

        self.pub_ground_truth = dict()
        self.pub_als = dict()
        self.pub_gps = dict()
        
        for i in range(self.path_group):
            self.pub_ground_truth[i] = rospy.Publisher(self.pub_ground_truth_topic + str(i+1), Path, queue_size = 20)
            self.pub_als[i] = rospy.Publisher(self.pub_als_topic + str(i+1), Path, queue_size = 20)
            self.pub_gps[i] = rospy.Publisher(self.pub_gps_topic + str(i+1), Path, queue_size = 20)

        # Subscribers
        self.sub_ground_truth = rospy.Subscriber("~ground_truth_posestamped", PoseStamped, self.cbBaseLine, queue_size=10)
        self.sub_als = rospy.Subscriber("~als_posestamped", PoseStamped, self.cbALS, queue_size=10)
        self.sub_gps = rospy.Subscriber("~gps_posestamped", PoseStamped, self.cbGPS, queue_size=10)
        self.channel = rospy.Subscriber("~channel", Int32, self.cbchannel, queue_size=1)



        """
        # Publishers
        self.pub_ground_truth = rospy.Publisher('~ground_truth_path', Path, queue_size = 20)
        self.pub_als = rospy.Publisher('~als_path', Path, queue_size = 20)
        self.pub_gps = rospy.Publisher('~gps_path', Path, queue_size = 20)

        # Publishers
        self.pub_ground_truth2 = rospy.Publisher('~ground_truth_path2', Path, queue_size = 20)
        self.pub_als2 = rospy.Publisher('~als_path2', Path, queue_size = 20)
        self.pub_gps2 = rospy.Publisher('~gps_path2', Path, queue_size = 20)

        # Publishers
        self.pub_ground_truth3 = rospy.Publisher('~ground_truth_path3', Path, queue_size = 20)
        self.pub_als3 = rospy.Publisher('~als_path3', Path, queue_size = 20)
        self.pub_gps3 = rospy.Publisher('~gps_path3', Path, queue_size = 20)

        # Publishers
        self.pub_ground_truth4 = rospy.Publisher('~ground_truth_path4', Path, queue_size = 20)
        self.pub_als4 = rospy.Publisher('~als_path4', Path, queue_size = 20)
        self.pub_gps4 = rospy.Publisher('~gps_path4', Path, queue_size = 20)

        # Publishers
        self.pub_ground_truth5 = rospy.Publisher('~ground_truth_path5', Path, queue_size = 20)
        self.pub_als5 = rospy.Publisher('~als_path5', Path, queue_size = 20)
        self.pub_gps5 = rospy.Publisher('~gps_path5', Path, queue_size = 20)
        """

    def cbBaseLine(self, posestamped):
        if(self.is_ground_truth_first == False):
            if self.channel > 0 and self.channel <= self.path_group:
                self.ground_truth_path_msg[self.channel-1].poses.append(posestamped)
                self.pub_ground_truth[self.channel-1].publish(self.ground_truth_path_msg[self.channel-1])
            """
            if self.channel == 1:
                self.ground_truth_path_msg.poses.append(posestamped)
                self.pub_ground_truth.publish(self.ground_truth_path_msg)
            if self.channel == 2:
                self.ground_truth_path_msg2.poses.append(posestamped)
                self.pub_ground_truth2.publish(self.ground_truth_path_msg2)
            if self.channel == 3:
                self.ground_truth_path_msg3.poses.append(posestamped)
                self.pub_ground_truth3.publish(self.ground_truth_path_msg3)
            if self.channel == 4:
                self.ground_truth_path_msg4.poses.append(posestamped)
                self.pub_ground_truth4.publish(self.ground_truth_path_msg4)
            if self.channel == 5:
                self.ground_truth_path_msg5.poses.append(posestamped)
                self.pub_ground_truth5.publish(self.ground_truth_path_msg5)
            """
        else:
            self.is_ground_truth_first = False
            self.ground_truth_first = posestamped.pose

    def cbALS(self, posestamped):
        if(self.is_als_first == False):
            posestamped.pose.position.x = posestamped.pose.position.x - self.als_first.position.x + self.ground_truth_first.position.x
            posestamped.pose.position.y = posestamped.pose.position.y - self.als_first.position.y + self.ground_truth_first.position.y
            posestamped.pose.position.z = posestamped.pose.position.z - self.als_first.position.z + self.ground_truth_first.position.z
            
            if self.channel > 0 and self.channel <= self.path_group:
                self.als_path_msg[self.channel-1].poses.append(posestamped)
                self.pub_als[self.channel-1].publish(self.als_path_msg[self.channel-1])
            """
            if self.channel == 1:
                self.als_path_msg.poses.append(posestamped)
                self.pub_als.publish(self.als_path_msg)
            if self.channel == 2:
                self.als_path_msg2.poses.append(posestamped)
                self.pub_als2.publish(self.als_path_msg2)
            if self.channel == 3:
                self.als_path_msg3.poses.append(posestamped)
                self.pub_als3.publish(self.als_path_msg3)
            if self.channel == 4:
                self.als_path_msg4.poses.append(posestamped)
                self.pub_als4.publish(self.als_path_msg4)
            if self.channel == 5:
                self.als_path_msg5.poses.append(posestamped)
                self.pub_als5.publish(self.als_path_msg5)
                """
        else:
            self.als_first = posestamped.pose
            self.is_als_first = False

    def cbGPS(self, posestamped):
        if(self.is_gps_first == False):
            posestamped.pose.position.x = posestamped.pose.position.x - self.gps_first.position.x + self.ground_truth_first.position.x
            posestamped.pose.position.y = posestamped.pose.position.y - self.gps_first.position.y + self.ground_truth_first.position.y
            posestamped.pose.position.z = posestamped.pose.position.z - self.gps_first.position.z + self.ground_truth_first.position.z

            if self.channel > 0 and self.channel <= self.path_group:
                self.gps_path_msg[self.channel-1].poses.append(posestamped)
                self.pub_gps[self.channel-1].publish(self.gps_path_msg[self.channel-1])

            """
            if self.channel == 1:
                self.gps_path_msg.poses.append(posestamped)
                self.pub_gps.publish(self.gps_path_msg)
            if self.channel == 2:
                self.gps_path_msg2.poses.append(posestamped)
                self.pub_gps2.publish(self.gps_path_msg2)
            if self.channel == 3:
                self.gps_path_msg3.poses.append(posestamped)
                self.pub_gps3.publish(self.gps_path_msg3)
            if self.channel == 4:
                self.gps_path_msg4.poses.append(posestamped)
                self.pub_gps4.publish(self.gps_path_msg4)
            if self.channel == 5:
                self.gps_path_msg5.poses.append(posestamped)
                self.pub_gps5.publish(self.gps_path_msg5)
                """
        else:
            self.gps_first = posestamped.pose
            self.is_gps_first = False

    def cbchannel(self, channel_msg):
        self.channel = channel_msg.data
        print "change channel to ", self.channel

#    def cb_path(self, pose):
#        pose_stamped = PoseStamped()
#        pose_stamped.header.frame_id = "odom"
#        pose_stamped.pose = pose
#        self.path_msg.header.stamp = rospy.Time.now()
#        self.path_msg.header.seq = pose_stamped.header.seq
#        pose_stamped.header.stamp = self.path_msg.header.stamp
#        self.path_msg.poses.append(pose_stamped)
#        self.pub_path.publish(self.path_msg)
#        print 'poses------------'
#        print self.path_msg.poses
#        print '------------'


#    def cbModelStates2Pose(self, model_states):
#        #print model_states
#        wamv_id = 0
#        for name in model_states.name:
#            if(name != 'wamv'):
#                wamv_id = wamv_id + 1
#            else:
#                if(self.verbose): print model_states.name[wamv_id], model_states.pose[wamv_id]
#                pose = Pose()
#                pose = model_states.pose[wamv_id]
#                pose.position.x = pose.position.x + 5
#                pose.position.y = pose.position.y - 5
#                self.cb_path(pose)


    def onShutdown(self):
        for i in range(self.path_group):
            self.write_ground_truth[i].close()
            self.write_gps[i].close()
            self.write_als[i].close()
        rospy.loginfo("[ModelStatePathNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('wamv_path',anonymous=False)
    wamv_path = WAMVPath()
    rospy.on_shutdown(wamv_path.onShutdown)
    rospy.spin()
