#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


class GPS(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose=False

        self.first = True
        self.first_latitude = 0
        self.first_longitude = 0
        self.first_altitude = 0

        # Subscribers
        self.sub_gps = rospy.Subscriber("/gps", NavSatFix, self.cbGPS, queue_size=1)

        # Publishers
        self.pub_gps = rospy.Publisher('~gps_posestamped', PoseStamped, queue_size = 20)

    def cbGPS(self, navsatfix):
        if(self.first == True):
            self.first = False
            self.first_latitude = navsatfix.latitude
            self.first_longitude = navsatfix.longitude
            #self.first_altitude = navsatfix.altitude
        pose_stamped = PoseStamped()
        pose_stamped.header = navsatfix.header
        pose_stamped.pose.position.x = navsatfix.longitude - self.first_longitude
        pose_stamped.pose.position.y = navsatfix.latitude - self.first_latitude
        #pose_stamped.pose.position.z = navsatfix.altitude - self.first_altitude
        pose_stamped.pose.position.x = pose_stamped.pose.position.x * 100000
        pose_stamped.pose.position.y = pose_stamped.pose.position.y * 100000
        pose_stamped.pose.position.z = 0
        self.pub_gps.publish(pose_stamped)
        if(self.verbose): print pose_stamped.pose


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
    rospy.init_node('gps',anonymous=False)
    gps = GPS()
    rospy.on_shutdown(gps.onShutdown)
    rospy.spin()
