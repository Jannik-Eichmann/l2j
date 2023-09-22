#!/usr/bin/env python3


"""
* GUI function *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""
import time
import tkinter as tk
import tkinter.ttk as ttk
import tkinter.font as font
import threading
import cv2
import PIL
import os
import rospy as rp
import sys
import numpy as np
from geometry_msgs.msg import Transform, Quaternion, Vector3
from PIL import Image, ImageTk
from tf import transformations as tft  # TODO: remove?
from scipy.spatial.transform import Rotation
from tkinter import messagebox, DISABLED, ACTIVE

# enable imports from folder "nodes"
user_home_path = os.path.expanduser("~")
node_dir_path = os.path.join(user_home_path, "ws_moveit", "src", "l2j", "nodes")
sys.path.insert(1, node_dir_path)
import camera_node


class GuiRobotApp:
    def __init__(self, master=None):
        os.system("gnome-terminal --tab -- bash -c 'roscore; exec bash'")
        # build ui
        # image Ratio 3(height) to 4(width)
        self.height = 810  # 810
        self.width = 1080  # 1080

        self.cameraNode = camera_node.Camera()

        # Aruco date stream in a seperat Thread.
        threading.Thread(
            target=self.cameraNode.stream_aruco_data, name="ArucoStream", daemon=True
        ).start()

        # threading.Thread(target=self.high, name="DepthDate", daemon=True).start()

        self.imageType = 2

        self.mainLevel = tk.Tk() if master is None else tk.Toplevel(master)
        self.mainLevel.title("LuLuJa Kamerakalibrationsprogramm")
        screen_width = self.mainLevel.winfo_screenwidth()
        screen_height = self.mainLevel.winfo_screenheight()
        self.mainLevel.resizable(False, False)
        self.mainLevel.geometry(
            str(int(screen_width / 2)) + "x" + str(int(screen_height * 0.8))
        )
        # self.mainLevel.minsize(1980, 1080)  # 1440, 810 1600, 900

        # Menu
        menubar = tk.Menu(self.mainLevel, background="white", fg="black")
        self.mainLevel.config(menu=menubar)
        # create new Menu item to handle files
        # self.file_menu = tk.Menu(menubar, tearoff=0)
        # self.file_menu.add_command(label="Neue Messung", command=self.reset)
        # self.file_menu.add_command(label="Messung speichern")
        # self.file_menu.add_command(label="Öffnen der letzte Messung")
        # self.file_menu.add_separator()
        # self.file_menu.add_command(label="Exit", command=lambda: exit())
        # menubar.add_cascade(label="File", menu=self.file_menu)

        # create new Menu Item to show impressum
        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="Impressum", command=self.impressum_call)
        help_menu.add_separator()
        help_menu.add_command(label="Exit", command=lambda: exit())
        menubar.add_cascade(label="Info", menu=help_menu)

        # self.mainLevel.resizable(False,False)
        self.interval = 20  # Interval in ms to get the lates

        # leftframe
        self.leftFrame = ttk.Frame(self.mainLevel)

        self.beenden_btn = ttk.Button(self.leftFrame)
        self.beenden_btn.configure(text="Beenden")
        self.beenden_btn.place(anchor="nw", relx=0.1, rely=0.95, x=0, y=0)
        self.beenden_btn.configure(command=self.close)

        self.manual_txt = tk.Text(self.leftFrame, font=("Helvetica", 10))
        self.manual_txt.configure(
            height=10,
            insertunfocussed="hollow",
            state="disabled",
            width=50,
        )
        _text_ = "1.Starten Sie den Roboter über das Teach-Panel\n2.Stellen Sie sicher, dass der ArUco Marker im Sichtfeld der Kamera ist.\n3.Starten Sie den Roboter.\n4.Starten Sie das Script 'external control' \nauf dem Teach-Panel.\n5.Starten Sie RViz und warten Sie bis \nMoveIt geladen ist.\n6.Starten Sie die Kalibrierung.\n7.Warten Sie bis die Transformations-\nmatrix ausgeben wird."
        self.manual_txt.configure(state="normal")
        self.manual_txt.insert("0.0", _text_)
        self.manual_txt.configure(state="disabled")
        self.manual_txt.place(
            anchor="nw", relheight=0.2, relwidth=0.8, relx=0.1, rely=0.1, x=0, y=0
        )

        # Frame that contains the buttons to start the configuration
        self.button_frame = ttk.Frame(self.leftFrame)
        self.button_frame.configure(height=200, width=200)

        self.robot_btn = ttk.Button(self.button_frame)
        self.robot_btn.configure(text="Starte Roboter", width=20)
        self.robot_btn.place(anchor="center", relx=0.5, rely=0.3)  # 0.25
        self.robot_btn.configure(
            command=lambda: threading.Thread(
                target=self.start_Robot_Ros, name="ros", daemon=True
            ).start()
        )

        info_lbl = ttk.Label(self.button_frame)
        info_lbl.configure(font="{Ubuntu} 12 {}", text="Setup Routine für den Roboter:")
        info_lbl.place(anchor="center", relx=0.5, rely=0.1)

        # start Rviz and Moveit
        self.rviz_btn = ttk.Button(self.button_frame)
        self.rviz_btn.configure(text="Starte RViz", state=DISABLED, width=20)
        self.rviz_btn.place(anchor="center", relx=0.5, rely=0.5)
        self.rviz_btn.configure(
            command=lambda: threading.Thread(
                target=self.start_rviz, name="startRivz", daemon=True
            ).start()
        )
        # start of the configuration.
        self.configure_cam_btn = ttk.Button(self.button_frame)
        self.configure_cam_btn.configure(
            text="Starte Kalibrierung", state=DISABLED, width=20
        )
        self.configure_cam_btn.place(anchor="center", relx=0.5, rely=0.7)
        self.configure_cam_btn.configure(
            command=lambda: threading.Thread(
                target=self.start_camera_configuration,
                name="configurationCam",
                daemon=True,
            ).start()
        )
        # Button to uplade the tm to the tf tree.
        self.upload_to_tf_btn = ttk.Button(self.button_frame)
        self.upload_to_tf_btn.configure(text="Broadcaste TM", state=DISABLED, width=20)
        self.upload_to_tf_btn.place(anchor="center", relx=0.5, rely=0.9)
        self.upload_to_tf_btn.configure(
            command=lambda: threading.Thread(
                target=self.load_tm_2_tf, name="uploadTm", daemon=True
            ).start()
        )

        self.button_frame.place(
            anchor="nw", relwidth=1, relheight=0.15, relx=0.0, rely=0.34, x=0, y=0
        )  # 0.34
        self.leftFrame.place(
            anchor="nw", relheight=1, relwidth=0.33, relx=0, rely=0, x=0, y=0
        )

        # right Frame
        self.rightFrame = ttk.Frame(self.mainLevel)

        # canvas to display the camera image.
        self.image_cnvs = tk.Canvas(self.rightFrame)
        self.image_cnvs.configure(
            background="#d3d3d3",
            borderwidth=3,
            height=self.height,  # self.height,
            relief="sunken",
            width=self.width,  # self.width,
        )
        self.image_cnvs.place(
            anchor="nw",
            bordermode="outside",
            relx=0.05,
            rely=0.1,
            x=0,
            y=0,
            # relheight=0.45,
            # relwidth=0.8,
        )

        # Frame which contains the calculated data
        self.data_frame = ttk.Frame(self.rightFrame)
        self.data_frame.configure(height=200, width=200)

        self.cur_robot_pos_lbl = ttk.Label(self.data_frame)
        self.cur_robot_pos_lbl.configure(
            font="{ubuntu} 13 {}", text="Berechnete Kameraposition:"
        )

        xPosLbl = 0.4
        xPosDisplayLbl = 0.45

        self.cur_robot_pos_lbl.place(anchor="nw", relx=xPosLbl, rely=0, x=0, y=0)

        # Label to display the calculated position of the robot .
        self.img_x_lbl = ttk.Label(self.data_frame)
        self.img_x_lbl.configure(text="X:")
        self.img_x_lbl.place(anchor="nw", relx=xPosLbl, rely=0.07, x=0, y=0)

        self.img_y_lbl = ttk.Label(self.data_frame)
        self.img_y_lbl.configure(text="Y:")
        self.img_y_lbl.place(anchor="nw", relx=xPosLbl, rely=0.14, x=0, y=0)

        self.img_z_lbl = ttk.Label(self.data_frame)
        self.img_z_lbl.configure(text="Z:")
        self.img_z_lbl.place(anchor="nw", relx=xPosLbl, rely=0.21, x=0, y=0)

        # label to display the Values.
        self.display_x_pos_lbl = ttk.Label(self.data_frame)
        self.cur_x_pos = tk.DoubleVar()
        self.display_x_pos_lbl.configure(textvariable=self.cur_x_pos)
        self.display_x_pos_lbl.place(
            anchor="nw", relx=xPosDisplayLbl, rely=0.07, x=0, y=0
        )

        self.display_y_pos_lbl = ttk.Label(self.data_frame)
        self.cur_y_pos = tk.DoubleVar()
        self.display_y_pos_lbl.configure(textvariable=self.cur_y_pos)
        self.display_y_pos_lbl.place(
            anchor="nw", relx=xPosDisplayLbl, rely=0.14, x=0, y=0
        )

        self.display_z_pos_lbl = ttk.Label(self.data_frame)
        self.cur_z_pos = tk.DoubleVar()
        self.display_z_pos_lbl.configure(textvariable=self.cur_z_pos)
        self.display_z_pos_lbl.place(
            anchor="nw", relx=xPosDisplayLbl, rely=0.21, x=0, y=0
        )
        # Frame to display the calculated transformationmatrix.
        self.matrix_frame = ttk.Frame(self.data_frame)
        self.matrix_frame.configure(borderwidth=2, padding=3, relief="sunken")

        self.transmatrix_lbl = ttk.Label(self.matrix_frame)
        self.transmatrix_lbl.configure(
            font="{ubuntu} 12 {}", text="Transformationsmatrix:"
        )
        self.transmatrix_lbl.place(anchor="nw", relx=0.0, rely=0)

        # labels to display transmatrix.
        self.trans_x_lbl = ttk.Label(self.matrix_frame)
        self.trans_x_lbl.configure(text="Translation X:")
        self.trans_x_lbl.place(anchor="nw", relx=0.0, rely=0.15)

        self.trans_y_lbl = ttk.Label(self.matrix_frame)
        self.trans_y_lbl.configure(text="Translation Y:")
        self.trans_y_lbl.place(anchor="nw", relx=0.0, rely=0.3)

        self.trans_z_lbl = ttk.Label(self.matrix_frame)
        self.trans_z_lbl.configure(text="Translation Z:")
        self.trans_z_lbl.place(anchor="nw", relx=0.0, rely=0.45)

        self.display_trans_x_lbl = ttk.Label(self.matrix_frame)
        self.x = tk.DoubleVar()
        self.display_trans_x_lbl.configure(textvariable=self.x)
        self.display_trans_x_lbl.place(anchor="nw", relx=0.5, rely=0.15)

        self.display_trans_y_lbl = ttk.Label(self.matrix_frame)
        self.y = tk.DoubleVar()
        self.display_trans_y_lbl.configure(textvariable=self.y)
        self.display_trans_y_lbl.place(anchor="nw", relx=0.5, rely=0.30)

        self.display_trans_z_lbl = ttk.Label(self.matrix_frame)
        self.z = tk.DoubleVar()
        self.display_trans_z_lbl.configure(textvariable=self.z)
        self.display_trans_z_lbl.place(anchor="nw", relx=0.5, rely=0.45)

        self.rot_x_lbl = ttk.Label(self.matrix_frame)
        self.rot_x_lbl.configure(text="Rotation X:")
        self.rot_x_lbl.place(anchor="nw", relx=0.0, rely=0.6)

        self.rot_y_lbl = ttk.Label(self.matrix_frame)
        self.rot_y_lbl.configure(text="Rotation Y:")
        self.rot_y_lbl.place(anchor="nw", relx=0.0, rely=0.75)

        self.rot_z_lbl = ttk.Label(self.matrix_frame)
        self.rot_z_lbl.configure(text="Rotation Z:")
        self.rot_z_lbl.place(anchor="nw", relx=0.0, rely=0.9)

        self.display_rotation_x_lbl = ttk.Label(self.matrix_frame)
        self.rot_x = tk.DoubleVar()
        self.display_rotation_x_lbl.configure(textvariable=self.rot_x)
        self.display_rotation_x_lbl.place(anchor="nw", relx=0.5, rely=0.6)

        self.display_rotation_y_lbl = ttk.Label(self.matrix_frame)
        self.rot_y = tk.DoubleVar()
        self.display_rotation_y_lbl.configure(textvariable=self.rot_y)
        self.display_rotation_y_lbl.place(anchor="nw", relx=0.5, rely=0.75)

        self.display_rotation_z_lbl = ttk.Label(self.matrix_frame)
        self.rot_z = tk.DoubleVar()
        self.display_rotation_z_lbl.configure(textvariable=self.rot_z)
        self.display_rotation_z_lbl.place(anchor="nw", relx=0.5, rely=0.9)

        self.matrix_frame.place(anchor="nw", relwidth=0.35, relheight=0.47)

        self.data_frame.place(
            anchor="nw", relheight=0.35, relwidth=0.9, relx=0.05, rely=0.65, x=0, y=0
        )

        self.img_change_frame = ttk.Frame(self.rightFrame)
        self.img_change_frame.configure(height=45, width=200)

        self.current_img_lbl = ttk.Label(self.img_change_frame)
        self.img_var = tk.StringVar(value="ArUcobild")
        self.current_img_lbl.configure(
            anchor="center", text="Farbbild", textvariable=self.img_var
        )
        self.current_img_lbl.place(anchor="nw", rely=0.0, x=70, y=0, width=130)

        self.back_btn = ttk.Button(self.img_change_frame)
        self.back_btn.configure(
            text="<--",
            state=ACTIVE,
            command=lambda: threading.Thread(
                target=self.change_image, args=(False,), name="changeImg", daemon=True
            ).start(),
        )
        self.back_btn.place(anchor="nw", relx=0.0, rely=0.0, x=0, y=0, width=55)

        self.forward_btn = ttk.Button(self.img_change_frame)
        self.forward_btn.configure(
            text="-->",
            state=ACTIVE,
            command=lambda: threading.Thread(
                target=self.change_image, args=(True,), name="changeImg", daemon=True
            ).start(),
        )
        self.forward_btn.place(anchor="nw", rely=0.0, x=220, y=0, width=55)

        self.img_change_frame.place(
            anchor="nw", relx=0.4, rely=0.58, x=0, y=0, relwidth=0.3
        )  # , relheight=0.05)

        self.rightFrame.place(
            anchor="nw", relheight=1, relwidth=0.67, relx=0.3333, rely=0.0
        )

        self.sperator_mdl = ttk.Separator(self.mainLevel)
        self.sperator_mdl.configure(orient="vertical")
        self.sperator_mdl.place(
            anchor="nw", relheight=0.8, relwidth=0.0, relx=0.33, rely=0.1, x=0, y=0
        )

        # Main widget
        self.mainwindow = self.mainLevel

    def run(self):
        """Starts Mainloop and the multithreading of the Canvas and the Update of the Window."""
        threading.Thread(target=self.keep_responsive, daemon=True).start()
        threading.Thread(target=self.update_image, name="image", daemon=True).start()
        self.mainwindow.mainloop()

    def start_Robot_Ros(self):
        # check ros
        self.robot_btn.configure(state=DISABLED)
        self.rviz_btn.configure(state=ACTIVE)
        os.system(
            'gnome-terminal --tab -- bash -c "roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.2 kinematics_config:=$(rospack find ur_calibration)/etc/ur3e_robot_calibration.yaml; exec bash"'
        )
        messagebox.showwarning(
            title="Script starten",
            message="Bitte starten Sie das external control Script auf dem Teachpanel des Roboter.",
        )

    def start_rviz(self):
        self.rviz_btn.configure(state=DISABLED)
        self.configure_cam_btn.configure(state=ACTIVE)
        os.system(
            'gnome-terminal --tab -- bash -c "roslaunch ur3_gripper_moveit_config demo_real_ur3.launch limited:=true sim:=false; exec bash"'
        )

    def start_camera_configuration(self):
        """Starts the calibrations files. And create subscriber to get the Matrix and the Cam pos."""

        # !self.configure_cam_btn.configure(state=DISABLED)
        os.system(
            'gnome-terminal --tab -- bash -c "roslaunch l2j calibration.launch; exec bash"'
        )
        threading.Thread(
            target=self.create_subscriber_for_transmatrix,
            name="Subscriber",
            daemon=True,
        ).start()
        self.create_subscriber_for_camerpos()

    def create_subscriber_for_transmatrix(self):
        rp.init_node("camera_node", anonymous=True)

        rp.Subscriber("/avg_tm", Transform, callback=self.get_transmatrix_data)
        rp.spin()

    def create_subscriber_for_camerpos(self):
        rp.init_node("camera_node", anonymous=True)

        rp.Subscriber("/cam_pos", Vector3, callback=self.get_camera_data)
        rp.spin()

    def load_tm_2_tf(self):
        # Load the TM into the Tf Tree to work with it.
        self.configure_cam_btn.configure(state=DISABLED)
        self.upload_to_tf_btn.configure(state=DISABLED)
        os.system(
            'gnome-terminal --tab -- bash -c "roslaunch l2j broadcast_transform.launch; exec bash"'
        )

    def get_transmatrix_data(self, data):
        # get the matrix data from the node.
        self.x.set(round(data.translation.x, 4))
        self.y.set(round(data.translation.y, 4))
        self.z.set(round(data.translation.z, 4))

        rot_quat = []
        rot_quat.append(data.rotation.x)
        rot_quat.append(data.rotation.y)
        rot_quat.append(data.rotation.z)
        rot_quat.append(data.rotation.w)

        rot_euler = tft.euler_from_quaternion(rot_quat, "sxyz")

        self.rot_x.set(round(rot_euler[0], 4))
        self.rot_y.set(round(rot_euler[1], 4))
        self.rot_z.set(round(rot_euler[2], 4))

        self.upload_to_tf_btn.configure(state=ACTIVE)

    def get_camera_data(self, data):
        self.cur_x_pos.set(round(data.x, 4))
        self.cur_y_pos.set(round(data.y, 4))
        self.cur_z_pos.set(round(data.z, 4))

    def close(self):
        """close the code."""
        if messagebox.askyesno(
            title="Beenden?", message="Wollen Sie das Program beenden?"
        ):
            try:
                for proc in psutil.process_iter():
                    try:
                        if "rosmaster" in proc.name().lower():
                            procdict = proc.as_dict(
                                attrs=["pid", "name", "create_time"]
                            )
                            print("Processs:   ", procdict["pid"])
                        print(proc.name())
                    except Exception as e:
                        pass
                procID = int(procdict["pid"])
                os.kill(procID, signal.SIGKILL)
                os.system("gnome-terminal 'killall gnome-terminal'")
            except Exception as e:
                pass

            exit()

    def keep_responsive(self):
        self.mainwindow.update()
        self.mainwindow.after(100, self.keep_responsive)

    def impressum_call(self):
        """Shows a messagebox with informations of the creator"""
        messagebox.showinfo(
            title="Impressum.",
            message="Diese Software ist das Eigentum der L2J Passing EMR Ltd. \nV.1.1 Sep 2023",
        )

    def update_image(self):
        """Takes Frames from the Videofotage and display it in the canvas."""
        # Quelle: https://scribles.net/showing-video-image-on-tkinter-window-with-opencv/
        # get the image from the nodes and Resize it
        self.image = self.cameraNode.get_image_data(imageType=self.imageType)
        self.image = cv2.resize(
            src=self.image,
            dsize=(self.width, self.height),
            interpolation=cv2.INTER_AREA,
        )
        self.image = Image.fromarray(self.image)  # to PIL format
        self.image = ImageTk.PhotoImage(self.image)  # to ImageTk format
        # Update image
        # insert the image in the Canvas.
        self.image_cnvs.create_image(0, 0, anchor=tk.NW, image=self.image)
        # Repeat every 'interval' ms
        self.mainwindow.after(self.interval, self.update_image)

    def change_image(self, nextImg):
        """Change the displayed Image from color to depth to aruco and back."""
        # imgType = self.img_var.get()
        if nextImg:
            self.imageType = self.imageType + 1
            if self.imageType > 2:
                self.imageType = 0
        elif not nextImg:
            self.imageType = self.imageType - 1
            if self.imageType < 0:
                self.imageType = 2

        if self.imageType == 0:
            self.img_var.set("Farbbild")

        elif self.imageType == 1:
            self.img_var.set("Tiefenbild")

        elif self.imageType == 2:
            self.img_var.set("ArUcobild")

    def reset(self):
        """Resets the setup buttons. not in use"""
        # os.system("gnome-terminal --tab -- bash -c 'roscore; exec bash'")
        # self.rviz_btn.configure(state=DISABLED)
        # self.configure_cam_btn.configure(state=DISABLED)
        # self.robot_btn.configure(state=ACTIVE)
        self.configure_cam_btn.configure(state=ACTIVE)


class MenuApp:
    def __init__(self, master=None) -> None:
        # window to chose to load matrix or to open the callibration window.
        self.menu_window = tk.Tk() if master is None else tk.Toplevel(master)
        self.menu_window.geometry("500x400")
        self.menu_window.title("LuLuJa Startmenü")
        self.menu_window.resizable(False, False)
        self.main_Frame = ttk.Frame(self.menu_window)

        self.close_btn = ttk.Button(self.main_Frame)
        self.close_btn.configure(text="Beenden", command=self.close)
        self.close_btn.place(anchor="center", relx=0.5, rely=0.8)

        self.header_lbl = ttk.Label(self.main_Frame)
        self.header_lbl.configure(text="Wählen Sie bitte eine Option aus:")
        self.header_lbl.place(anchor="center", relx=0.5, rely=0.05)

        self.load_matrix_btn = ttk.Button(self.main_Frame)
        self.load_matrix_btn.configure(
            text="Broadcaste neueste TM", command=self.load_matrix
        )
        self.load_matrix_btn.place(anchor="center", relx=0.5, rely=0.20)

        self.load_config_prog_btn = ttk.Button(self.main_Frame)
        self.load_config_prog_btn.configure(
            text="Starte Kalibrationsprogramm", command=self.start_callibration_prog
        )
        self.load_config_prog_btn.place(anchor="center", relx=0.5, rely=0.38)

        self.main_Frame.place(anchor="nw", relheight=1, relwidth=1)
        self.mainWindow = self.menu_window

    def start_callibration_prog(self):
        self.mainWindow.destroy()
        app = GuiRobotApp()
        app.run()

    def load_matrix(self):
        self.load_config_prog_btn.configure(state=DISABLED)
        self.load_matrix_btn.configure(state=DISABLED)
        os.system(
            'gnome-terminal --tab -- bash -c "roslaunch l2j broadcast_transform.launch; exec bash"'
        )

    def close(self):
        """close the code."""

        if messagebox.askyesno(
            title="Beenden?", message="Wollen Sie das Program beenden?"
        ):
            quit(0)

    def run(self):
        """Starts Mainloop"""
        self.mainWindow.mainloop()


if __name__ == "__main__":
    app = MenuApp()
    # app = GuiRobotApp()
    app.run()
