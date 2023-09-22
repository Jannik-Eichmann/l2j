#!/usr/bin/env python3

"""
* Matrix Calculator Node *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import cv2
import math
import numpy as np
import rospy as rp
import tf
from _parent_node import Node
from l2j.srv import TMCalculation, TMCalculationRequest, TMCalculationResponse
from geometry_msgs.msg import Transform, Vector3, Quaternion
from scipy.spatial.transform import Rotation


class MatrixCalculator(Node):
    def __init__(self):
        super().__init__("matrix_calculator")
        self.create_service_server(
            "transformation_matrix_calculation",
            TMCalculation,
            self.calculate_transformation_matrix,
        )

    def calculate_transformation_matrix(self, request: TMCalculationRequest):
        if not request.aruco_data.marker_detected:
            return TMCalculationResponse(False, Vector3(), Vector3())

        # get coordinates of aruco vertices in pixel coordinates
        aruco_pixel_coords = np.array(
            [
                [request.aruco_data.corner_0.x, request.aruco_data.corner_0.y],
                [request.aruco_data.corner_1.x, request.aruco_data.corner_1.y],
                [request.aruco_data.corner_2.x, request.aruco_data.corner_2.y],
                [request.aruco_data.corner_3.x, request.aruco_data.corner_3.y],
            ],
            dtype=np.double,
        )

        # get coordinates of aruco vertices in robot coordinates
        aruco_robot_coords = np.array(
            [
                [
                    request.robot_data.aruco_corner_0.x,
                    request.robot_data.aruco_corner_0.y,
                    request.robot_data.aruco_corner_0.z,
                ],
                [
                    request.robot_data.aruco_corner_1.x,
                    request.robot_data.aruco_corner_1.y,
                    request.robot_data.aruco_corner_1.z,
                ],
                [
                    request.robot_data.aruco_corner_2.x,
                    request.robot_data.aruco_corner_2.y,
                    request.robot_data.aruco_corner_2.z,
                ],
                [
                    request.robot_data.aruco_corner_3.x,
                    request.robot_data.aruco_corner_3.y,
                    request.robot_data.aruco_corner_3.z,
                ],
            ],
            dtype=np.double,
        )

        # get camera intrinsics
        intrinsics = np.array(
            [
                [
                    request.aruco_data.intrinsics_fx,
                    0,
                    request.aruco_data.intrinsics_ppx,
                ],
                [
                    0,
                    request.aruco_data.intrinsics_fy,
                    request.aruco_data.intrinsics_ppy,
                ],
                [0, 0, 1],
            ],
            dtype=np.double,
        )

        # camera distortion coefficients are constant 0
        distortion_coefficients = np.array([[0, 0, 0, 0]], dtype=np.float32)

        # try transformation calculation five times
        for _ in range(5):
            # get transformation matrix from pixel coordinates to robot coordinates
            successful, rotation, translation = cv2.solvePnP(
                objectPoints=aruco_robot_coords,
                imagePoints=aruco_pixel_coords,
                cameraMatrix=intrinsics,
                distCoeffs=distortion_coefficients,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            # return transformation matrix on success
            if successful:
                translation = Vector3(translation[0], translation[1], translation[2])
                rotation = Vector3(rotation[0], rotation[1], rotation[2])

                return TMCalculationResponse(successful, translation, rotation)

        return TMCalculationResponse(False, Vector3(), Vector3())


if __name__ == "__main__":
    matrix_calculator = MatrixCalculator()

    matrix_calculator.spin()
