#!/usr/bin/env python3

"""
* Robot Control Node *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import numpy as np
import rospy as rp
import random
import tf
import sys
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
from _parent_node import Node
from l2j.srv import MoveRobot, MoveRobotRequest, MoveRobotResponse
from l2j.msg import ArUcoData
from time import sleep
from scipy.spatial.transform import Rotation


class RobotControl(Node):
    __robot: moveit_commander.RobotCommander
    __scene: moveit_commander.PlanningSceneInterface
    __move_group: moveit_commander.MoveGroupCommander

    # storage for initial values of joint angles
    __initial_joint_values: list = []

    # range limit of joint movement
    __RANGE: float = 0.3

    def __init__(self):
        # initialize ros node
        super().__init__("robot_control_node")

        # initialize the moveit commander, robot commander and move group commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.__robot = moveit_commander.RobotCommander()
        self.__move_group = moveit_commander.MoveGroupCommander("ur3_arm")

        # store initial joint values
        self.__initial_joint_values = self.__move_group.get_current_joint_values()

        # instantiate an interface to the world surrounding the robot
        self.__scene = moveit_commander.PlanningSceneInterface()

        print(self)

        self.create_service_server("/move_robot", MoveRobot, self.calibration_movement)
        rp.loginfo("Initialized robot_control_node")

    def __str__(self):
        # move group information: reference frame, name of end-effector link
        reference_frame = self.__move_group.get_planning_frame()
        eef_link = self.__move_group.get_end_effector_link()

        # robot information: group names
        group_names = self.__robot.get_group_names()

        return (
            f"[INFO] {self.__class__.__name__}\n"
            + f"\tPlanning Frame:\t\t\t{reference_frame}\n"
            + f"\tEnd Effector Link:\t\t{eef_link}\n"
            + f"\tAvailable Planning Groups:\t{group_names}\n"
        )

    def calibration_movement(self, request: MoveRobotRequest):
        self.go_to_random_position()

        # needed to prevent robot from standing still caused by
        # delay when aruco marker is not detected anymore
        rp.sleep(0.5)

        # move robot to new position when aruco marker is not detected in current position
        aruco_data = rp.wait_for_message("/aruco_data_stream", ArUcoData)
        while not aruco_data.marker_detected:
            self.go_to_random_position()
            aruco_data = rp.wait_for_message("/aruco_data_stream", ArUcoData)

        return MoveRobotResponse(True)

    def go_to_random_position(self):
        joint_goals = []
        for i in range(4):
            joint_goals.append(self.__initial_joint_values[i])

        # change only joint values of the last two joints to prevent destructive movements
        joint_goals.append(
            self.__initial_joint_values[4] + random.uniform(-self.__RANGE, self.__RANGE)
        )
        joint_goals.append(
            self.__initial_joint_values[5] + random.uniform(-self.__RANGE, self.__RANGE)
        )

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or
        # joint target for the group
        self.__move_group.go(joint_goals, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.__move_group.stop()


if __name__ == "__main__":
    robot_control_node = RobotControl()

    robot_control_node.spin()
