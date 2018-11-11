#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates


class WAMVPath(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose=False

        # mapping all path
        self.is_ground_truth_first = True
        self.is_als_first = True
        self.is_gps_first = True
        self.ground_truth_first = Pose()
        self.als_first = Pose()
        self.pgs_first = Pose()

        # apriltag localization path
        self.grame_id = "odom"
        self.ground_truth_path_msg = Path()
        self.ground_truth_path_msg.header.frame_id = self.grame_id
        self.gps_path_msg = Path()
        self.gps_path_msg.header.frame_id = self.grame_id
        self.als_path_msg = Path()
        self.als_path_msg.header.frame_id = self.grame_id

        # Subscribers
        self.sub_ground_truth = rospy.Subscriber("~ground_truth_posestamped", PoseStamped, self.cbBaseLine, queue_size=1)
        self.sub_als = rospy.Subscriber("~als_posestamped", PoseStamped, self.cbALS, queue_size=1)
        self.sub_gps = rospy.Subscriber("~gps_posestamped", PoseStamped, self.cbGPS, queue_size=1)

        # Publishers
        self.pub_ground_truth = rospy.Publisher('~ground_truth_path', Path, queue_size = 20)
        self.pub_als = rospy.Publisher('~als_path', Path, queue_size = 20)
        self.pub_gps = rospy.Publisher('~gps_path', Path, queue_size = 20)

    def cbBaseLine(self, posestamped):
        if(self.is_ground_truth_first == False):
            self.ground_truth_path_msg.poses.append(posestamped)
            self.pub_ground_truth.publish(self.ground_truth_path_msg)
        else:
            self.is_ground_truth_first = False
            self.ground_truth_first = posestamped.pose

    def cbALS(self, posestamped):
        if(self.is_als_first == False):
            posestamped.pose.position.x = posestamped.pose.position.x - self.als_first.position.x + self.ground_truth_first.position.x
            posestamped.pose.position.y = posestamped.pose.position.y - self.als_first.position.y + self.ground_truth_first.position.y
            posestamped.pose.position.z = posestamped.pose.position.z - self.als_first.position.z + self.ground_truth_first.position.z
            self.als_path_msg.poses.append(posestamped)
            self.pub_als.publish(self.als_path_msg)
        else:
            self.als_first = posestamped.pose
            self.is_als_first = False

    def cbGPS(self, posestamped):
        if(self.is_gps_first == False):
            posestamped.pose.position.x = posestamped.pose.position.x - self.gps_first.position.x + self.ground_truth_first.position.x
            posestamped.pose.position.y = posestamped.pose.position.y - self.gps_first.position.y + self.ground_truth_first.position.y
            posestamped.pose.position.z = posestamped.pose.position.z - self.gps_first.position.z + self.ground_truth_first.position.z
            self.gps_path_msg.poses.append(posestamped)
            self.pub_gps.publish(self.gps_path_msg)
        else:
            self.gps_first = posestamped.pose
            self.is_gps_first = False


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
        rospy.loginfo("[ModelStatePathNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('wamv_path',anonymous=False)
    wamv_path = WAMVPath()
    rospy.on_shutdown(wamv_path.onShutdown)
    rospy.spin()
