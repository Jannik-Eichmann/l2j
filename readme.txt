README

This README describes the installation and usage of l2j package and the necessary changes to config files as of 2023-09-22
This package is intended for usage with Universal Robots UR3e, RobotIQ 2F-85 Gripper and Intel RealSense D435.
Any other usage needs to be tested and is not guaranteed to yield successful results.



##### Installation #####
1. Clone github repository to "~/ws_moveit/src"
2. Change to the working directory by using "cd ws_moveit/src/l2j" and create folder "saved_tms" by using "mkdir saved_tms".
3. Make necessary changes described below in paragraph "Neccessary changes to files from EMR repository"
4. Open terminal in "~/ws_moveit" and enter command "catkin build"
5. Install dependencies described in "requirements.txt" in folder "scripts" AND "nodes".
6. Hardware: Please attach ArUco marker to the UR3e gripper. The adapter needs to be attached opposite
   of any connection cables/adapters of the gripper.
7. If you want to change the ArUco marker make sure, that the ArUco markers origin matches with the adapters marked corner.
8. Ready to use, have fun ...



##### USAGE #####
To start the GUI open a new terminal an type
"rosrun l2j gui_script.py"

Choose whether to broadcast the latest transformation matrix to the tf-tree or to start a new calibration.
If you choose to a new calibration run, follow these steps:
1. Make sure, that the adapter is positioned correctly on the gripper of the robot.
2. Make sure the robot is running and connected via wire to the PC
3. Press "Starte Kalibrationsprogramm"
4. Position the robot by using the free-drive-mode, so that the corners of the ArUco marker
   are visible in the GUI and marked with black dots. You can change between three types of
   images using the arrows beneath the camer image.
5. Press "Starte Roboter" and follow the pop-up instructions. 
   (Start "external_control" script on the robots teach panel.)
6. Press "Starte RViz" and wait for MoveIt to initialize.
7. Press "Starte Kalibrierung", wait for the completion of the calbration program.
8. The transformation has been completed and saved in the "saved_tms" directory.
9. You can broadcast the results by pressing "Broadcaste TM" or broadcast them later 
   by using the GUIs starting page.
10. To close the GUI you can press "Beenden" or "Info". To stop the nodes, please close the terminals.
11. Further informations can be found in the impressum under "Info". 



##### Neccessary changes to files from EMR repository #####

The positions of the aruco marker vertices is manually calculated. To add the aruco marker positions in the tf tree go to:

/home/luluja/ws_moveit/src/fmauch_universal_robot/ur_description/urdf/ur3_robotiq85_gripper.urdf.xacro
deactivated Joint limits in urdf file
  <!-- arm -->
  <xacro:ur3_robot prefix="" joint_limited="false"
    shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
    elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />

and add the following joints:

<joint name="aruco_joint_0" type="fixed">
    <origin xyz="0.025 0.0407 0.0523" rpy="0 0 ${-pi/2.0}" />
    <parent link="tool0"/>   							  
    <child link="aruco_link_0"/>
</joint>

<link name="aruco_link_0" />

<joint name="aruco_joint_1" type="fixed">
    <origin xyz="-0.025 0.0407 0.0523" rpy="0 0 ${-pi/2.0}" />
    <parent link="tool0"/>   							  
    <child link="aruco_link_1"/>
</joint>

<link name="aruco_link_1" />

<joint name="aruco_joint_2" type="fixed">
    <origin xyz="-0.025 0.0407 0.0023" rpy="0 0 ${-pi/2.0}" />
    <parent link="tool0"/>   							  
    <child link="aruco_link_2"/>
</joint>

<link name="aruco_link_2" />

<joint name="aruco_joint_3" type="fixed">
    <origin xyz="0.025 0.0407 0.0023" rpy="0 0 ${-pi/2.0}" />
    <parent link="tool0"/>   							  
    <child link="aruco_link_3"/>
</joint>

<link name="aruco_link_3" />