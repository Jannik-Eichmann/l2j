#!/usr/bin/env python3

"""
* Camera Node *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import cv2
import numpy as np
import rospy as rp
from _parent_node import Node
from _camera import RealSenseCamera
from l2j.msg import ArUcoData
from geometry_msgs.msg import Pose2D
from random import randint as rng


class Camera(Node):
    __camera: RealSenseCamera
    __publisher: rp.Publisher
    __rate: rp.Rate

    def __init__(self):
        # ros publisher node setup
        super().__init__(name="camera_node")
        self.__publisher, self.__rate = self.create_publisher(
            "aruco_data_stream", ArUcoData
        )

        # camera setup
        self.__camera = RealSenseCamera()
        self.__camera.setup_stream()

    def stream_aruco_data(self):
        while not rp.is_shutdown():
            self.__camera.record_scene()
            self.__camera.process_images()
            marker_detected, aruco_vertices = self.__camera.detect_aruco_markers()

            if marker_detected:
                fx, fy, ppx, ppy = self.__camera.get_intrinsics()
                aruco_data = ArUcoData(
                    marker_detected,
                    fx,
                    fy,
                    ppx,
                    ppy,
                    Pose2D(aruco_vertices[0, 0], aruco_vertices[0, 1], 0),
                    Pose2D(aruco_vertices[1, 0], aruco_vertices[1, 1], 0),
                    Pose2D(aruco_vertices[2, 0], aruco_vertices[2, 1], 0),
                    Pose2D(aruco_vertices[3, 0], aruco_vertices[3, 1], 0),
                )
                self.__publisher.publish(aruco_data)
            else:
                # publish "empty" message if no aruco marker is detected
                # -> should enable robot to move to alternative position
                fx, fy, ppx, ppy = self.__camera.get_intrinsics()
                aruco_data = ArUcoData(
                    marker_detected,
                    fx,
                    fy,
                    ppx,
                    ppy,
                    Pose2D(0, 0, 0),
                    Pose2D(0, 0, 0),
                    Pose2D(0, 0, 0),
                    Pose2D(0, 0, 0),
                )
                self.__publisher.publish(aruco_data)

    def stream_test_data(self):
        while not rp.is_shutdown():
            # simulate aruco marker detection
            self.__aruco_data.marker_detected = True

            # transmit dummy values for camera intrinsics
            self.__aruco_data.intrinsics_fx = 300
            self.__aruco_data.intrinsics_fy = 300
            self.__aruco_data.intrinsics_ppx = 320
            self.__aruco_data.intrinsics_ppy = 240

            # randomise coordinates of aruco vertices
            self.__aruco_data.corner_0 = Pose2D(rng(0, 640), rng(0, 480), 0)
            self.__aruco_data.corner_1 = Pose2D(rng(0, 640), rng(0, 480), 0)
            self.__aruco_data.corner_2 = Pose2D(rng(0, 640), rng(0, 480), 0)
            self.__aruco_data.corner_3 = Pose2D(rng(0, 640), rng(0, 480), 0)

            # publish to aruco data stream
            self.publish(self.__aruco_data, self.__publisher, self.__rate)

    def get_image_data(self, imageType: int):
        self.__camera.record_scene()
        self.__camera.process_images()
        self.__camera.detect_aruco_markers()

        image = self.__camera.get_image(imageType)
        return image

    def get_depth_info(self):
        coordinates = self.__camera.get_aruco_in_cam_coords()
        return coordinates


if __name__ == "__main__":
    camera_node = Camera()

    camera_node.stream_aruco_data()
