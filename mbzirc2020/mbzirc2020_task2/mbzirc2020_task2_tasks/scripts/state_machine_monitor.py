#!/usr/bin/env python

import rospy
from smach_msgs.msg import SmachContainerStatus
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft
import tf2_ros
import numpy as np
import ros_numpy


class StateMachineMonitor:
    def __init__(self):


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.state = None
        self.level_cam_global_trans = None
        self.baselink2cam_trans = None
        self.baselink_worldcoords = None
        self.level_cam_worldcoords = None


        self.state_machine_status_sub = rospy.Subscriber('/hydrus/task2_smach_server/smach/container_status', SmachContainerStatus, self.stateMachineStateCallback)

        self.baselink_odom_sub_ = rospy.Subscriber('/hydrus/uav/baselink/odom', Odometry, self.baselinkOdomCallback)
        self.object_global_sub = rospy.Subscriber('/hydrus/task2_motion/target_pose', PoseStamped, self.objectPoseCallback)
        self.object_local_pos_pub = rospy.Publisher('/hydrus/task2_motion/grasp/object/local_pos', Vector3Stamped, queue_size = 1)

        # channel pos relay
        self.channel_pos_sub = rospy.Subscriber('/hydrus/channel_center/pos', Vector3Stamped, self.channelPosCallback)
        self.place_channel_pos_pub = rospy.Publisher('/hydrus/place/channel_center/pos', Vector3Stamped, queue_size = 1)
        self.ungrasp_channel_pos_pub = rospy.Publisher('/hydrus/ungrasp/channel_center/pos', Vector3Stamped, queue_size = 1)



    def baselinkOdomCallback(self, msg):
        self.baselink_worldcoords = ros_numpy.numpify(msg.pose.pose)


        # if not self.state == 'Grasp':
        #     return

        if self.baselink2cam_trans is None:
            try:
                self.baselink2cam_trans = self.tf_buffer.lookup_transform('hydrus/fc',
                                                                          'hydrus/rs_d435_color_optical_frame',
                                                                          rospy.Time.now(),
                                                                          rospy.Duration(1.0))
                self.baselink2cam_trans = ros_numpy.numpify(self.baselink2cam_trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                    return

        cam_worldcoords = tft.concatenate_matrices(self.baselink_worldcoords, self.baselink2cam_trans)
        yaw = tft.euler_from_matrix(cam_worldcoords)[2]
        pos = tft.translation_from_matrix(cam_worldcoords)
        self.level_cam_worldcoords = tft.concatenate_matrices(tft.translation_matrix(pos),
                                                         tft.euler_matrix(0, 0, yaw + np.pi/2))

    def objectPoseCallback(self, msg):

        if self.level_cam_worldcoords is None:
            return

        local_pos = tft.translation_from_matrix(
            tft.concatenate_matrices(tft.inverse_matrix(self.level_cam_worldcoords),
                                     ros_numpy.numpify(msg.pose)))

        local_pos_msg = Vector3Stamped()
        local_pos_msg.header.stamp = msg.header.stamp
        local_pos_msg.vector.x = local_pos[0]
        local_pos_msg.vector.y = local_pos[1]
        local_pos_msg.vector.z = local_pos[2]
        self.object_local_pos_pub.publish(local_pos_msg)


    def channelPosCallback(self, msg):
        if self.state == 'PlaceVisualServoing':
            # print("place")
            self.place_channel_pos_pub.publish(msg)

        if self.state == 'Ungrasp':
            # print("ungrasp")
            self.ungrasp_channel_pos_pub.publish(msg)


    def stateMachineStateCallback(self, msg):

        if msg.path == '/SM_ROOT':
            return

        if msg.active_states[0] == 'None':
            return

        self.state = msg.active_states[0]

        # rospy.loginfo("current state: {}".format(self.state))

if __name__=="__main__":

    rospy.init_node('state_machine_replay')

    monitor = StateMachineMonitor()

    rospy.spin()


