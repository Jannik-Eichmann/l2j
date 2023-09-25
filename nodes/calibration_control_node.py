#!/usr/bin/env python3

"""
* Calibration Control Node *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import cv2
import math
import message_filters
import numpy as np
import os
import rosbag
import rospy as rp
import tf
from datetime import datetime
from geometry_msgs.msg import Transform, Quaternion, Vector3
from l2j.msg import ArUcoData, RobotData
from l2j.srv import TMCalculation, MoveRobot, MoveRobotRequest
from _parent_node import Node
from scipy.spatial.transform import Rotation


class CalibrationControl(Node):
    __publisher: rp.Publisher
    __rate: rp.Rate

    # storage for translation and rotation vectors of the current sample
    __sample_translations: list = []
    __sample_rotations: list = []

    # storage for translation and rotation vectors of the averaged samples
    __average_translations: list = []
    __average_rotations: list = []

    # storage for final translation and rotation vector
    __final_translation: np.ndarray
    __final_rotation: np.ndarray

    # storage for final average transformation matrix
    __final_tm: Transform = None
    __camera_pos: Vector3 = None
    __camera_rot: Vector3 = None

    # sample sizes for single position and positions
    __SAMPLES = 10
    __SAMPLES_AVERAGE = 10

    def __init__(self):
        super().__init__("calculation_control")

        self.__tm_publisher, self.__rate = self.create_publisher(
            "/avg_tm", Transform, rate=10
        )

        self.__cam_pos_publisher, self.__cam_pos_rate = self.create_publisher(
            "/cam_pos", Vector3, rate=1
        )

        aruco_stream_subscriber = message_filters.Subscriber(
            "aruco_data_stream", ArUcoData
        )

        robot_stream_subscriber = message_filters.Subscriber(
            "robot_data_stream", RobotData
        )

        time_synced_subscriber = message_filters.ApproximateTimeSynchronizer(
            [aruco_stream_subscriber, robot_stream_subscriber],
            queue_size=10,
            slop=0.1,
            allow_headerless=True,
        )

        time_synced_subscriber.registerCallback(self.calculate_transformation_matrix)

    def calculate_transformation_matrix(self, aruco_data, robot_data):
        response_tm = self.create_service_client(
            "transformation_matrix_calculation",
            TMCalculation,
            aruco_data,
            robot_data,
        )

        # publish final transformation matrix if enough samples of
        # average transformation matrices have been collected
        if len(self.__average_translations) == self.__SAMPLES_AVERAGE:
            self.publish_tm()
            return

        if response_tm.success and (len(self.__sample_translations) < self.__SAMPLES):
            # in case of successful calculation store transformation matrix in list
            current_translation = [
                response_tm.translation.x,
                response_tm.translation.y,
                response_tm.translation.z,
            ]

            current_rotation = [
                response_tm.rotation.x,
                response_tm.rotation.y,
                response_tm.rotation.z,
            ]

            self.__sample_translations.append(current_translation)
            self.__sample_rotations.append(current_rotation)

            return

        if len(self.__sample_translations) == self.__SAMPLES:
            # calculate average transformation matrix
            avg_translation, avg_rotation = self.calculate_average_tm(
                self.__sample_translations, self.__sample_rotations
            )

            self.__average_translations.append(avg_translation)
            self.__average_rotations.append(avg_rotation)

            self.__sample_translations: list = []
            self.__sample_rotations: list = []

            # request movement to new position
            please_move: bool = True
            movement_success = self.create_service_client(
                "/move_robot", MoveRobot, please_move
            )

    def calculate_average_tm(self, translations: list, rotations: list):
        # when samplesize is reached, calculate average and publish
        translations = np.asarray(translations)
        rotations = np.asarray(rotations)

        # calculate average of translation and rotation
        avg_translation = np.median(translations, axis=0)
        avg_rotation = np.median(rotations, axis=0)

        return (avg_translation, avg_rotation)

    @staticmethod
    def get_camera_position(translation: np.ndarray, rotation: np.ndarray):
        camera_translation = translation[:, np.newaxis]
        camera_rotation = rotation[:, np.newaxis]

        rotation_matrix, _ = cv2.Rodrigues(camera_rotation)

        cam_trans = -np.matrix(rotation_matrix).T * np.matrix(camera_translation)
        cam_trans = Vector3(cam_trans[0], cam_trans[1], cam_trans[2])

        cam_rot = Vector3(camera_rotation[0], camera_rotation[1], camera_rotation[2])
        return cam_trans, cam_rot

    def publish_tm(self):
        # check if __final_tm has been calculated
        if self.__final_tm is None:
            self.calculate_final_tm()
            rp.loginfo("__final_tm:")
            rp.loginfo(self.__final_tm)
            rp.loginfo("__camera_pos")
            rp.loginfo(self.__camera_pos)

        # publish average tranformation matrix
        self.publish(self.__final_tm, self.__tm_publisher, self.__rate)

        # publish camera position
        self.publish(self.__camera_pos, self.__cam_pos_publisher, self.__cam_pos_rate)

    def calculate_final_tm(self):
        # calculate the translation and rotation for the __final_tm
        self.__final_translation, self.__final_rotation = self.calculate_average_tm(
            self.__average_translations, self.__average_rotations
        )

        # camera position is calculated as control value to check
        # validity of transformation matrix
        self.__camera_pos, self.__camera_rot = CalibrationControl.get_camera_position(
            self.__final_translation, self.__final_rotation
        )

        # cast numpy ndarrays to Vector3 and Quaternion message types
        # to combine them into Transform message type
        translation_vector = Vector3(
            self.__final_translation[0],
            self.__final_translation[1],
            self.__final_translation[2],
        )

        rotation_vector = tf.transformations.quaternion_from_euler(
            self.__camera_rot.x,
            self.__camera_rot.y,
            self.__camera_rot.z,
            "sxyz",
        )

        rotation_vector = Quaternion(
            rotation_vector[0],
            rotation_vector[1],
            rotation_vector[2],
            rotation_vector[3],
        )

        # store average transformation matrix
        self.__final_tm = Transform(translation_vector, rotation_vector)

        # write average transformation matrix to file for later usage
        self.write_tm_to_file(self.__final_tm, self.__camera_pos)

    def write_tm_to_file(self, tm, cam_pos):
        # get current date and time
        current_date_time = str(datetime.now())
        current_date_time = current_date_time.replace(":", "-")
        current_date_time = current_date_time.replace(" ", "_")

        # get path of this files directory
        # file_dir_path = os.path.dirname(os.path.abspath(__file__))
        user_home_path = os.path.expanduser("~")

        trans_mat_file_name = "trans_mat_world2cam_" + current_date_time + ".bag"

        # construct path of the transformation matrix data file
        # format: /home/user/ws_moveit/src/l2j/config/tf_camera2base.txt
        trans_mat_file_path = os.path.join(
            user_home_path, "ws_moveit", "src", "l2j", "saved_tms", trans_mat_file_name
        )

        bag = rosbag.Bag(trans_mat_file_path, "w")

        tm_data2file = Transform()
        tm_data2file.translation = cam_pos
        tm_data2file.rotation = tm.rotation

        try:
            bag.write("/avg_tm", tm_data2file)

        finally:
            bag.close()
            rp.loginfo("Data written to bag file. Filepath:")
            rp.loginfo(trans_mat_file_path)
        return


if __name__ == "__main__":
    calculation_control = CalibrationControl()

    calculation_control.spin()
