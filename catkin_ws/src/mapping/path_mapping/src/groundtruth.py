#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates


class GroundTruth(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.verbose=False

        # Subscribers
        self.sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, self.cbModelStates2PoseStamped, queue_size=1)

        # Publishers
        self.pub_ground_truth = rospy.Publisher('~ground_truth_posestamped', PoseStamped, queue_size = 20)

    def cbModelStates2PoseStamped(self, model_states):
        #print model_states
        wamv_id = 0
        for name in model_states.name:
            if(name != 'wamv'):
                wamv_id = wamv_id + 1
            else:
                if(self.verbose): print model_states.name[wamv_id], model_states.pose[wamv_id]
                pose_stamped = PoseStamped()
                pose_stamped.pose = model_states.pose[wamv_id]
                pose_stamped.header.frame_id = "odom"
                pose_stamped.header.stamp = rospy.Time.now()
                self.pub_ground_truth.publish(pose_stamped)


    def onShutdown(self):
        rospy.loginfo("[ModelStatePathNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('grpund_truth',anonymous=False)
    grpund_truth = GroundTruth()
    rospy.on_shutdown(grpund_truth.onShutdown)
    rospy.spin()
