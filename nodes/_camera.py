#!/usr/bin/env python3

"""
* RealSense Camera Class *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import cv2
from cv2 import aruco
import numpy as np
import pyrealsense2 as rs

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": aruco.DICT_APRILTAG_36h11,
}


class RealSenseCamera:
    # structure objects
    __pipeline: rs.pipeline
    __pipeline_profile: rs.pipeline_profile
    __config: rs.config
    __device: rs.device
    __align: rs.align

    # camera information
    __is_streaming: bool = False
    __intrinsics: tuple

    # depth information
    __depth_frame: rs.depth_frame
    __depth_image: np.ndarray

    # color information
    __color_frame: rs.video_frame
    __color_image: np.ndarray

    # point cloud
    __point_cloud: rs.pointcloud
    __point_cloud_points: rs.points
    __point_coords_xyz: np.ndarray

    # aruco marker
    __marker_coords_uv: np.ndarray
    __marker_coords_xyz: np.ndarray

    def __init__(self):
        # create a pipeline
        self.__pipeline = rs.pipeline()

        # creates a config and configure the pipeline to stream different resolutions of color and depth streams
        self.__config = rs.config()

        # ? config class reference:
        # ? https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1config.html

        # create a wrapper and matches device and stream profiles
        pipeline_wrapper = rs.pipeline_wrapper(self.__pipeline)
        self.__pipeline_profile = self.__config.resolve(pipeline_wrapper)

        # ? pipeline_profile class reference
        # ? https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1pipeline__profile.html

        # retrieve the device used by the pipeline
        self.__device = self.__pipeline_profile.get_device()

        # ? device class reference:
        # ? https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1device.html

    def __str__(self):
        # get device information
        name = str(self.__device.get_info(rs.camera_info.name))
        product_line = str(self.__device.get_info(rs.camera_info.product_line))
        product_id = str(self.__device.get_info(rs.camera_info.product_id))
        serial_number = str(self.__device.get_info(rs.camera_info.serial_number))

        return (
            f"< Camera >\n"
            + f"\tName:\t{name}\n"
            + f"\tProduct Line:\t{product_line}\n"
            + f"\tProduct ID:\t{product_id}\n"
            + f"\tSerial Number:\t{serial_number}\n"
            + f""
        )

    def setup_stream(self):
        """
        Method to enable depth and color data streaming. This needs to be called prior the streaming loop.
        """

        # check for correct camera type
        found_rgb = False
        for s in self.__device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # enable depth data stream and color data stream
        self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # start streaming
        self.__pipeline_profile = self.__pipeline.start(self.__config)

        self.__is_streaming = True

        # get camera intrinsics from stream
        profile = self.__pipeline_profile.get_stream(rs.stream.color)
        intrinsics = profile.as_video_stream_profile().get_intrinsics()

        self.__intrinsics = (
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.ppx,
            intrinsics.ppx,
        )

    def align(self, align_to_depth: np.bool_ = False):
        """
        The camera streams are captured from different viewports. Spatial alignment allows to
        translate from one viewport to another. In aligned streams several artifacts occure:
        * Sampling: aligning may cause downsampling or upsampling (via interpolation) to match different resolutions.
        * Occlussion: aligning depth sensors to rgb sensors may cause invalid depth values, because these points were occluded from the depth sensors viewport.

        ? https://dev.intelrealsense.com/docs/rs-align

        Args:
            align_to_depth (np.bool_, optional): enables alignment to depth frame. Beware the severe occlussion! Defaults to False.
        """

        if align_to_depth:
            align_to = rs.stream.depth
        else:
            align_to = rs.stream.color

        # create an align object
        self.__align = rs.align(align_to)

        # ? align class reference:
        # ? https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1align.html

    def record_scene(self, get_pixel_coords: np.bool_ = False):
        # wait for a coherent pair of frames (frameset): depth and color
        frames = self.__pipeline.wait_for_frames()

        # align the frames as selected in aling_to
        aligned_frames = frames

        # all distortion coefficients are zero
        # ? https://github.com/IntelRealSense/librealsense/issues/1430#issuecomment-375945916

        # retrieve  depth and color frome fropm aligned frames
        self.__depth_frame = aligned_frames.get_depth_frame().as_depth_frame()
        self.__color_frame = aligned_frames.get_color_frame()

    def get_intrinsics(self):
        return self.__intrinsics

    def get_point_cloud(self):
        # initialize a point cloud object
        point_cloud = rs.pointcloud()

        # initialize points object
        pcl_points = rs.points()

        # calculate point cloud points from depth frame
        pcl_points = point_cloud.calculate(self.__depth_frame)

        # calculate cartesian point coordinates
        # ? source: https://github.com/IntelRealSense/librealsense/issues/6749#issuecomment-654185205
        pcl_points_xyz = pcl_points.get_vertices()
        pcl_points_xyz = np.asanyarray(pcl_points_xyz).view(np.float32).reshape(-1, 3)
        return pcl_points_xyz

    def process_images(self):
        # convert frames to numpy arrays / images
        depth_image = np.asanyarray(self.__depth_frame.get_data())
        color_image = np.asanyarray(self.__color_frame.get_data())

        # compare frame dimensions and rescale to match shapes, if necessary
        depth_image_shape = depth_image.shape
        color_image_shape = color_image.shape

        # match color image shape to depth image shape
        # ? color image upscaling uses inter area interpolation
        if depth_image_shape != color_image_shape:
            color_image = cv2.resize(
                color_image,
                dsize=(depth_image_shape[1], depth_image_shape[0]),
                interpolation=cv2.INTER_AREA,
            )
        else:
            pass

        self.__depth_image = depth_image
        self.__color_image = color_image

    def get_color_image(self):
        return self.__color_image

    def get_depth_image(self):
        return self.__depth_image

    def detect_aruco_markers(self, dictionary: str = "DICT_6X6_1000"):
        # convert color image to open cv grayscale image
        self.__aruco_image = cv2.cvtColor(self.__color_image, cv2.COLOR_BGR2GRAY)

        # setup aruco detector
        aruco_dictionary = aruco.Dictionary_get(ARUCO_DICT[dictionary])
        detector_parameters = aruco.DetectorParameters_create()

        # get bounding boxes of all aruco markers in the fov
        bounding_boxes, _, _ = aruco.detectMarkers(
            self.__aruco_image, aruco_dictionary, parameters=detector_parameters
        )

        marker_detected = bool(len(bounding_boxes))

        if marker_detected:
            # initialize binary image of the vertices of all bounding boxes
            aruco_bitmask = np.zeros(self.__aruco_image.shape, dtype=np.float32)

            try:
                # convert bounding_boxes to numpy array with all vertex point coordinates uv
                aruco_vertices = np.stack(bounding_boxes, axis=0).astype(np.uint16)
                aruco_vertices = np.squeeze(aruco_vertices, axis=1)
                aruco_vertices = np.concatenate(aruco_vertices, axis=0)

                # set all vertex pixels to 1
                aruco_bitmask[aruco_vertices[:, 1], aruco_vertices[:, 0]] = 255

            except Exception as e:
                pass

            # draw the marker vertices in the grayscale image
            try:
                for vertex in aruco_vertices:
                    cv2.circle(
                        img=self.__aruco_image,
                        center=(
                            vertex[0],
                            vertex[1],
                        ),
                        radius=3,
                        color=(0, 0, 0),
                        thickness=3,
                    )
                aruco.drawDetectedMarkers(self.__aruco_image, bounding_boxes)

            except Exception as e:
                pass
            return marker_detected, aruco_vertices
        return marker_detected, 0

    def show(self):
        # Show images
        window_name = "RealSense Color Image"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(window_name, self.__color_image)

        window_name = "RealSense Depth Map"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(window_name, self.__depth_image)

        window_name = "ArUco Grayscale Image"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(window_name, self.__aruco_image)

        # press esc or 'q' to close the image window and to stop streaming
        key = cv2.waitKey(1)
        if key & 0xFF == ord("q") or key == 27:
            cv2.destroyAllWindows()

    def get_image(self, imgType: int):
        if imgType == 0:
            # return color Image
            self.image = cv2.cvtColor(self.__color_image, cv2.COLOR_BGR2RGB)
            return self.image
        elif imgType == 1:
            # return depth Image
            return self.__depth_image
        elif imgType == 2:
            self.image = cv2.cvtColor(self.__aruco_image, cv2.COLOR_BGR2RGB)
            return self.image

    def get_aruco_in_cam_coords(self):
        marker_detected, aruco_vertices = self.detect_aruco_markers()

        if marker_detected:
            # get camera intrinsics from stream
            intrinsics = rs.video_stream_profile(
                self.__depth_frame.get_profile()
            ).get_intrinsics()

            # calculate 3d coordinates from pixel coordinates and pixel distances (in meters)
            # ? https://github.com/IntelRealSense/librealsense/issues/2458

            points_3d = []

            for x, y in aruco_vertices.tolist():
                pixel_distance = self.__depth_frame.get_distance(x, y)
                point_3d = rs.rs2_deproject_pixel_to_point(
                    intrinsics, [float(x), float(y)], pixel_distance
                )

                points_3d.append(point_3d)

            return np.array(points_3d)

        return None
