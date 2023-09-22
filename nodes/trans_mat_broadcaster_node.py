#!/usr/bin/env python3

"""
* Trans Mat Broadcaster Node *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""


import cv2
import numpy as np
import os
import rosbag
import rospy as rp
import tf
from visualization_msgs.msg import Marker


class TransMatBroadcaster:
    def __init__(self):
        # initialize broadcaster node
        rp.init_node("broadcast_tf_world2cam")

        # read data of transformation matrix from file
        translation, rotation = self.read_trans_mat_file()

        # publish the transformation matrix
        self.broadcast_tm(translation, rotation)

    def read_trans_mat_file(self):
        # get path of this files directory
        # file_dir_path = os.path.dirname(os.path.abspath(__file__))
        user_home_path = os.path.expanduser("~")

        # construct path of the transformation matrix data file
        # format: /home/user/ws_moveit/src/l2j/config/trans_mat_world2cam_YEAR-MONTH-DAY_ROSTIME.msgpack
        trans_mat_dir_path = os.path.join(
            user_home_path, "ws_moveit", "src", "l2j", "saved_tms"
        )

        # get all path of the saved transformation files
        trans_mat_filenames = os.listdir(trans_mat_dir_path)
        trans_mat_file_paths = []
        for filename in trans_mat_filenames:
            trans_mat_file_paths.append(os.path.join(trans_mat_dir_path, filename))

        # get only newest file path
        trans_mat_file_path = max(trans_mat_file_paths, key=os.path.getctime)

        # construct file path
        rp.loginfo("Reading data from file:")
        rp.loginfo(trans_mat_file_path)

        # read data from bag file
        bag = rosbag.Bag(trans_mat_file_path)

        for topic, msg, t in bag.read_messages(topics=["/avg_tm"]):
            # read translation from saved Transform
            translation = [msg.translation.x, msg.translation.y, msg.translation.z]
            # read rotation from saved Transform
            rotation = [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
            rp.loginfo("Read following transformation: ")
            rp.loginfo(msg)
        bag.close()
        return translation, rotation

    def broadcast_tm(self, trans_base2camera, rot_base2camera):
        br = tf.TransformBroadcaster()
        rate = rp.Rate(1.0)

        # publish the transformation matrix from the read file
        while not rp.is_shutdown():
            br.sendTransform(
                trans_base2camera, rot_base2camera, rp.Time.now(), "camera", "world"
            )
            rp.loginfo("sending transform /world => /camera")
            rate.sleep()


if __name__ == "__main__":
    tm_broadcaster = TransMatBroadcaster()
