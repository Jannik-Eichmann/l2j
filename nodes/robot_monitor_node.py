#!/usr/bin/env python3

"""
* Robot Monitor Node *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import numpy as np
import rospy as rp
from _parent_node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from l2j.msg import RobotData
from random import randint as rng
import tf


class RobotMonitor(Node):
    __publisher: rp.Publisher
    __rate: rp.Rate
    __robot_data: RobotData
    __tf_listener: tf.TransformListener

    __aruco_tf_links = [
        "aruco_link_0",
        "aruco_link_1",
        "aruco_link_2",
        "aruco_link_3",
    ]

    def __init__(self):
        # ros publisher node setup
        self.__robot_data = RobotData()
        super().__init__("robot_monitor")
        self.__publisher, self.__rate = self.create_publisher(
            "robot_data_stream", RobotData
        )

        # create a transform listener
        # ? http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
        self.__tf_listener = tf.TransformListener()
        for link in self.__aruco_tf_links:
            self.__tf_listener.waitForTransform(
                link,
                "/base_link",
                rp.Time(),
                rp.Duration(5.0),
            )

        self.__subscriber = self.create_subscriber(
            "/joint_states", JointState, self.stream_robot_data
        )

    def stream_robot_data(self, joint_states: JointState):
        # get the aruco  in which x, y and z coordinate will be stored
        aruco_robot_coords = []
        try:
            for link in self.__aruco_tf_links:
                (translation, _) = self.__tf_listener.lookupTransform(
                    "base_link", link, rp.Time(0)
                )
                aruco_robot_coords.append(translation)
        except:
            pass

        self.__robot_data.robot_moving = np.any(joint_states.velocity)
        self.__robot_data.aruco_corner_0 = Point(
            aruco_robot_coords[0][0],
            aruco_robot_coords[0][1],
            aruco_robot_coords[0][2],
        )
        self.__robot_data.aruco_corner_1 = Point(
            aruco_robot_coords[1][0],
            aruco_robot_coords[1][1],
            aruco_robot_coords[1][2],
        )
        self.__robot_data.aruco_corner_2 = Point(
            aruco_robot_coords[2][0],
            aruco_robot_coords[2][1],
            aruco_robot_coords[2][2],
        )
        self.__robot_data.aruco_corner_3 = Point(
            aruco_robot_coords[3][0],
            aruco_robot_coords[3][1],
            aruco_robot_coords[3][2],
        )

        self.publish(self.__robot_data, self.__publisher)

    def stream_test_data(self):
        while not rp.is_shutdown():
            # simulate aruco marker detection
            self.__robot_data.robot_moving = False

            # simulate aruco marker positions
            self.__robot_data.aruco_corner_0 = Point(
                rng(0, 100), rng(0, 100), rng(0, 100)
            )
            self.__robot_data.aruco_corner_1 = Point(
                rng(0, 100), rng(0, 100), rng(0, 100)
            )
            self.__robot_data.aruco_corner_2 = Point(
                rng(0, 100), rng(0, 100), rng(0, 100)
            )
            self.__robot_data.aruco_corner_3 = Point(
                rng(0, 100), rng(0, 100), rng(0, 100)
            )

            # publish to robot data stream
            self.publish(self.__robot_data, self.__publisher, self.__rate)


if __name__ == "__main__":
    robot_monitor = RobotMonitor()

    robot_monitor.spin()
