#!/usr/bin/env python3

# Copyright 2022 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
# and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or other materials provided
# with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
# endorse or promote products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from pyntcloud import PyntCloud
import sys
import threading
import termios
import fcntl
from math import sin, cos, tan, pi, sqrt
from tf import transformations
import imageio
import time
import os
import numpy as np
import pandas as pd
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo, RegionOfInterest
from std_msgs.msg import Int16MultiArray, Float32MultiArray, MultiArrayLayout, MultiArrayDimension, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point, Pose
import tf2_ros
from collections import defaultdict
import xmltodict
import itertools
from kitti360_publisher.msg import Kitti360BoundingBox, Kitti360SemanticID, Kitti360SemanticRGB, Kitti360InstanceID, Kitti360Confidence
from labels import id2label, name2label


class Kitti360DataPublisher:
    NODENAME = "kitti360_publisher"
    DESIRED_RATE = 100  # Hz

    # Data paths
    DATA_DIRECTORY = None
    SEQUENCE = None
    SEQUENCE_DIRECTORY = None

    # ------------------------------------------
    # ROS Publishers
    # - publish_* is always a bool defining whether a specific resource is
    #   supposed to be loaded/published (automatically disabled if resource not
    #   available)
    # - ros_publisher_* is the respective publisher

    # clock
    ros_publisher_clock = None

    # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/data_rect/{frame:0>10}.png
    ros_publisher_2d_raw_perspective_rectified_left = None
    publish_perspective_rectified_left = None
    ros_publisher_2d_raw_perspective_rectified_right = None
    publish_perspective_rectified_right = None

    # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/data_rgb/{frame:0>10}.png
    ros_publisher_2d_raw_perspective_unrectified_left = None
    publish_perspective_unrectified_image_left = None
    ros_publisher_2d_raw_perspective_unrectified_right = None
    publish_perspective_unrectified_image_right = None

    # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{02|03}}/data_rgb/{frame:0>10}.png
    ros_publisher_2d_raw_fisheye_left = None
    publish_fisheye_left = None
    ros_publisher_2d_raw_fisheye_right = None
    publish_fisheye_right = None

    # data_2d_semantics/train/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/semantic/{frame:0>10}.png
    ros_publisher_2d_semantics_semantic_left = None
    publish_semantics_semantic_left = None
    ros_publisher_2d_semantics_semantic_right = None
    publish_semantics_semantic_right = None

    # data_2d_semantics/train/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/semantic_rgb/{frame:0>10}.png
    ros_publisher_2d_semantics_semantic_rgb_left = None
    publish_semantics_semantic_rgb_left = None
    ros_publisher_2d_semantics_semantic_rgb_right = None
    publish_semantics_semantic_rgb_right = None

    # data_2d_semantics/train/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/instance/{frame:0>10}.png
    ros_publisher_2d_semantics_instance_left = None
    publish_semantics_instance_left = None
    ros_publisher_2d_semantics_instance_right = None
    publish_semantics_instance_right = None

    # data_2d_semantics/train/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/confidence/{frame:0>10}.png
    ros_publisher_2d_semantics_confidence_left = None
    publish_semantics_confidence_left = None
    ros_publisher_2d_semantics_confidence_right = None
    publish_semantics_confidence_right = None

    # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/velodyne_points/data/{frame:0>10}.bin
    ros_publisher_3d_raw_velodyne = None
    publish_velodyne = None
    # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/sick_points/data/{frame:0>10}.bin
    ros_publisher_3d_raw_sick_points = None
    publish_sick_points = None

    # data_3d_semantics/train/2013_05_28_drive_{seq:0>4}_sync/static/{start_frame:0>10}_{end_frame:0>10}.ply
    ros_publisher_3d_semantics_static = None
    publish_3d_semantics_static = None
    bounds_3d_sem_static_index = None
    filename_3d_semantics_static = None
    records_3d_semantics_static = None

    # data_3d_semantics/train/2013_05_28_drive_{seq:0>4}_sync/dynamic/{start_frame:0>10}_{end_frame:0>10}.ply
    ros_publisher_3d_semantics_dynamic = None
    publish_3d_semantics_dynamic = None
    bounds_3d_sem_dynamic_index = None
    filename_3d_semantics_dynamic = None
    records_3d_semantics_dynamic = None

    # data_3d_semantics/train_full/2013_05_28_drive_{seq:0>4}_sync.xml
    previous_published_index_bb = None
    ros_publisher_bounding_boxes = None
    publish_bounding_boxes = None
    # rviz marker
    previous_published_index_bb_rviz = None
    ros_publisher_bounding_boxes_rviz_marker = None
    publish_bounding_boxes_rviz_marker = None
    # filled in self.read_bounding_boxes
    bounding_box_data = None
    bounding_box_frame_ranges = None

    # camera intrinsics
    ros_publisher_camera_intrinsics_unrectified_left = None
    ros_publisher_camera_intrinsics_unrectified_right = None
    ros_publisher_camera_intrinsics_fisheye_left = None
    ros_publisher_camera_intrinsics_fisheye_right = None
    publish_camera_intrinsics = None

    # ------------------------------------------
    # Timestamps

    # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/timestamps.txt
    timestamps_perspective_camera_00 = None
    timestamps_perspective_camera_01 = None
    # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{02|03}/timestamps.txt
    timestamps_fisheye_camera_02 = None
    timestamps_fisheye_camera_03 = None

    # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/velodyne_points/timestamps.txt
    timestamps_velodyne = None
    # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/sick_points/timestamps.txt
    timestamps_sick_points = None

    # ------------------------------------------
    # Poses

    # data_poses/2013_05_28_drive_{seq:0>4}_sync/poses.txt
    poses = None

    # NOTE: ignoring OXTS measurements

    # ------------------------------------------
    # SIMULATOR

    # configuration
    SIM_START = None
    SIM_END = None
    sim_playback_speed = None
    sim_looping = None

    # simulation loop variables
    last_published_frame = None
    sim_clock = None
    # sick needs a separate variable because data is published at different
    # rate
    last_published_sick_frame = None

    # stores the time when the simulation was resumed
    # simulation clock is computed based on that
    system_time_simulation_resumed = None
    # stores what the simulation time was when the simulation was last paused
    sim_to_resume_at = None

    # simulator state
    sim_paused = None

    # rviz player
    player_status = None

    # whether to print duration of each step (it is recorded anyway, NOTE, could
    # be changed)
    print_step_duration = False

    # total stats
    total_number_of_frames_velodyne = None
    total_number_of_frames_sick = None
    total_simulation_time = None

    # ------------------------------------------

    def __init__(self):
        # init ros node
        rospy.init_node(self.NODENAME)

        rospy.loginfo("   ___ ___ ___ ___   _   _   _             _   _   __")
        rospy.loginfo("|/  |   |   |   | __ _) |_  / \   o ._    |_) / \ (_ ")
        rospy.loginfo("|\ _|_  |   |  _|_   _) |_) \_/   | | |   | \ \_/ __)")

        padding = 17
        rospy.loginfo(
            "+----------------------------------------------------------------------+"
        )
        rospy.loginfo(f"{'node':<{padding}} {self.NODENAME}")

        # ------------------------------------------
        # get configuration parameters
        self.SIM_START = rospy.get_param("/kitti360_player/start", 0)
        rospy.loginfo(f"{'start:':<{padding}} {self.SIM_START}")
        self.SIM_END = rospy.get_param("/kitti360_player/end", 99999999)
        rospy.loginfo(f"{'end:':<{padding}} {self.SIM_END}")
        self.sim_playback_speed = rospy.get_param("/kitti360_player/rate", 1)
        rospy.loginfo(
            f"{'speed multiplier:':<{padding}} {self.sim_playback_speed}")

        self.SEQUENCE = rospy.get_param("/kitti360_player/sequence", 0)
        if not (self.SEQUENCE >= 0 and self.SEQUENCE <= 10):
            rospy.logerr(
                "sequence ({self.SEQUENCE=}) needs to be 0 <= x <= 10. FATAL")
            rospy.signal_shutdown("sequence number is invalid")
            exit()
        else:
            self.SEQUENCE = '{:04d}'.format(self.SEQUENCE)
            self.SEQUENCE_DIRECTORY = f"2013_05_28_drive_{self.SEQUENCE}_sync"
            rospy.loginfo(f"{'sequence:':<{padding}} {self.SEQUENCE}")
            rospy.loginfo(
                "+----------------------------------------------------------------------+"
            )

        self.DATA_DIRECTORY = rospy.get_param("kitti360_player/directory", "")
        if self.DATA_DIRECTORY == "" or not os.path.exists(
                self.DATA_DIRECTORY):
            rospy.logerr(
                f"Directory does not exist: {self.DATA_DIRECTORY}. FATAL.")
            rospy.logerr(
                "Note that the directory has to be specified as absolute path."
            )
            rospy.signal_shutdown("provided data directory is invalid")
            exit()

        self.sim_looping = rospy.get_param("kitti360_player/looping", True)

        # ------------------------------------------
        # what to enable/disable
        self.publish_velodyne = rospy.get_param(
            "/kitti360_player/pub_velodyne")
        self.publish_sick_points = rospy.get_param(
            "/kitti360_player/pub_sick_points")
        self.publish_perspective_rectified_left = rospy.get_param(
            "/kitti360_player/pub_perspective_rectified_left")
        self.publish_perspective_rectified_right = rospy.get_param(
            "/kitti360_player/pub_perspective_rectified_right")
        self.publish_perspective_unrectified_left = rospy.get_param(
            "/kitti360_player/pub_perspective_unrectified_left")
        self.publish_perspective_unrectified_right = rospy.get_param(
            "/kitti360_player/pub_perspective_unrectified_right")
        self.publish_fisheye_left = rospy.get_param(
            "/kitti360_player/pub_fisheye_left")
        self.publish_fisheye_right = rospy.get_param(
            "/kitti360_player/pub_fisheye_right")
        self.publish_bounding_boxes = rospy.get_param(
            "/kitti360_player/pub_bounding_boxes")
        self.publish_bounding_boxes_rviz_marker = rospy.get_param(
            "/kitti360_player/pub_bounding_boxes_rviz_marker")
        self.publish_semantics_semantic_left = rospy.get_param(
            "/kitti360_player/pub_2d_semantics_left")
        self.publish_semantics_semantic_right = rospy.get_param(
            "/kitti360_player/pub_2d_semantics_right")
        self.publish_semantics_semantic_rgb_left = rospy.get_param(
            "/kitti360_player/pub_2d_semantics_rgb_left")
        self.publish_semantics_semantic_rgb_right = rospy.get_param(
            "/kitti360_player/pub_2d_semantics_rgb_right")
        self.publish_semantics_instance_left = rospy.get_param(
            "/kitti360_player/pub_2d_instance_left")
        self.publish_semantics_instance_right = rospy.get_param(
            "/kitti360_player/pub_2d_instance_right")
        self.publish_semantics_confidence_left = rospy.get_param(
            "/kitti360_player/pub_2d_confidence_left")
        self.publish_semantics_confidence_right = rospy.get_param(
            "/kitti360_player/pub_2d_confidence_right")
        self.publish_3d_semantics_static = rospy.get_param(
            "/kitti360_player/pub_3d_semantics_static")
        self.publish_3d_semantics_dynamic = rospy.get_param(
            "/kitti360_player/pub_3d_semantics_dynamic")
        self.publish_camera_intrinsics = rospy.get_param(
            "/kitti360_player/pub_camera_intrinsics")

        rospy.loginfo(
            "Filling caches and preprocessing... this can take few seconds!")
        # ------------------------------------------
        # read timestamps for all data (multiple timestamps.txt)
        self.read_timestamps()

        # read poses
        self.read_poses()

        # init all publishers
        self.init_publishers()

        # read bounding boxes (max 72MB for a sequence)
        self.read_bounding_boxes()

        # create index and determine which frames are in what file
        self.read_3d_semantics_dir()

        # ------------------------------------------
        # init loop variables

        # simulation initially paused
        self.sim_paused = True
        self.sim_step = False

        # set clock to start time
        self.sim_clock = Clock()
        self.sim_to_resume_at = self.SIM_START

        # ------------------------------------------
        # publish camera intrinsics
        self.init_and_publish_camera_intrinsics()

        # ------------------------------------------
        # terminal simulation control
        input_thread = threading.Thread(target=self.terminal_sim_control)
        input_thread.daemon = True
        input_thread.start()

    # ------------------------------------------
    # MAIN LOOP
    def run(self):
        rospy.loginfo("starting simulation")

        # start simulation immediately
        self.unpause()

        while not rospy.is_shutdown():

            # store so that we know how long to sleep to match DESIRED_RATE
            system_time_loop_start = time.time()

            if not self.sim_paused and self._on_last_frame():
                rospy.loginfo("END OF SEQUENCE!")
                if self.sim_looping:
                    rospy.loginfo("Looping enabled --> back to start")
                    self._seek(self.SIM_START)
                else:
                    self.pause()

            # if the simulation is paused we do nothing
            if not self.sim_paused:
                self._simulation_update()

            # update ROS time and player
            self.ros_publisher_clock.publish(self.sim_clock)

            # compute how long we need to sleep to match DESIRED_RATE
            loop_duration = time.time() - system_time_loop_start
            if loop_duration < 1 / self.DESIRED_RATE:
                time.sleep(1 / self.DESIRED_RATE - loop_duration)

    def _simulation_update(self):
        # update simulation clock based on when the simulation was last
        # resumed
        # --> (ts when sim was resumed) + (timedelta since then it was resumed)
        self.sim_clock.clock = rospy.Time.from_sec(
            self.sim_to_resume_at +
            (time.time() - self.system_time_simulation_resumed) *
            self.sim_playback_speed)

        # for benchmarking
        sim_update_durations = dict()

        # need to be handles separately because of higher refresh rate
        sim_update_durations.update(self.handle_sick_points_publishing())

        next_frame = self._get_frame_to_be_published()
        # -1 --> sim time before first frame
        # using != instead of > to make seeking easier
        if next_frame != -1 and next_frame != self.last_published_frame:
            skipped = next_frame - self.last_published_frame - 1
            logfunc = rospy.logwarn if skipped > 0 else rospy.loginfo
            if skipped >= 0:
                skipped_string = f"(skipping {skipped})"
            else:
                skipped_string = "(backwards)"

            logfunc(
                f"new VELODYNE frame" + f" {skipped_string:<14}" +
                f"{self._convert_frame_int_to_string(self.last_published_frame)}"
                + f" -> {self._convert_frame_int_to_string(next_frame)} " +
                f"({self.sim_clock.clock.to_sec():.2f}s, {((self.sim_clock.clock.to_sec()/self.total_simulation_time)*100):.1f}%)"
            )

            # publish everything that is available
            sim_update_durations.update(
                self.publish_available_data(next_frame))

            # save frame that was just published
            self.last_published_frame = next_frame

            # for benchmarking --> how long each step took
            padding = 26
            if self.print_step_duration:
                rospy.loginfo(
                    "-------------------------------------------------------")
                total = sum(sim_update_durations.values())
                # print sorted by the duration
                for name, duration in sorted(sim_update_durations.items(),
                                             key=lambda x: x[1],
                                             reverse=True):
                    rospy.loginfo(
                        f"{name:<{padding}} = {duration:.3f}s ({((duration/total)*100):.1f}%)"
                    )
                rospy.loginfo(f"{'':<{padding}} ----------------")
                rospy.loginfo(f"{'total':<{padding}} = {total:.3f}s")
                rospy.loginfo(
                    "-------------------------------------------------------")
        elif next_frame == -1:
            # --> simulation timestamp is before first frame
            # setting this to -1 so that stepping works
            self.last_published_frame = -1

    def publish_available_data(self, frame):
        ret = dict()
        ret.update(self._publish_velodyne(frame))
        ret.update(self._publish_transforms(frame))
        ret.update(self._publish_images(frame))
        ret.update(self._publish_bounding_boxes(frame))
        ret.update(self._publish_bounding_boxes_rviz_marker(frame))
        ret.update(self._publish_2d_semantics(frame))
        ret.update(self._publish_3d_semantics(frame))
        return ret

    # ------------------------------------------
    # INIT
    def init_and_publish_camera_intrinsics(self):
        # CAM 00 (Perspective Left)
        ci_msg_00 = CameraInfo()
        ci_msg_00.header.seq = 0
        # not sure what to put here. doc says acquisition time of image
        ci_msg_00.header.stamp = rospy.Time(0)
        ci_msg_00.header.frame_id = "kitti360_cam_00"

        ci_msg_00.width = 1392
        ci_msg_00.height = 512

        ci_msg_00.distortion_model = "plumb_bob"
        ci_msg_00.K = [[788.629315, 0.000000, 687.158398],
                       [0.000000, 786.382230, 317.752196],
                       [0.000000, 0.000000, 0.000000]]

        ci_msg_00.D = [-0.344441, 0.141678, 0.000414, -0.000222, -0.029608]

        # NOTE this could be wrong, we are not sure. Please check
        # calibrations/perspective.txt
        ci_msg_00.R = [[1.000000, 0.000000, 0.000000],
                       [0.000000, 1.000000, 0.000000],
                       [0.000000, 0.000000, 1.000000]]

        ci_msg_00.P = [[552.554261, 0.000000, 682.049453, 0.000000],
                       [0.000000, 552.554261, 238.769549, 0.000000],
                       [0.000000, 0.000000, 1.000000, 0.000000]]

        self.ros_publisher_camera_intrinsics_unrectified_left.publish(
            ci_msg_00)

        # CAM 01 (Pespective Right)
        ci_msg_01 = CameraInfo()
        ci_msg_01.header.seq = 0
        ci_msg_01.header.stamp = rospy.Time(0)
        ci_msg_01.header.frame_id = "kitti360_cam_01"

        ci_msg_01.width = 1392
        ci_msg_01.height = 512

        ci_msg_01.distortion_model = "plumb_bob"
        ci_msg_01.K = [[785.134093, 0.000000, 686.437073],
                       [0.000000, 782.346289, 321.352788],
                       [0.000000, 0.000000, 0.000000]]

        ci_msg_01.D = [-0.353195, 0.161996, 0.000383, -0.000242, -0.041476]

        # NOTE this could be wrong, we are not sure. Please check
        # calibrations/perspective.txt
        ci_msg_01.R = [[0.999837, 0.004862, -0.017390],
                       [-0.004974, 0.999967, -0.006389],
                       [0.017358, 0.006474, 0.999828]]

        ci_msg_01.P = [[552.554261, 0.000000, 682.049453, -328.318735],
                       [0.000000, 552.554261, 238.769549, 0.000000],
                       [0.000000, 0.000000, 1.000000, 0.000000]]

        self.ros_publisher_camera_intrinsics_unrectified_right.publish(
            ci_msg_01)

        # FISHEYE
        # NOTE assumed that:
        # k3 = 0
        # cx = u0, cy = v0
        # gamma1 = fx, gamma2 fy

        # CAM 02 (Fisheye Left)
        ci_msg_02 = CameraInfo()
        ci_msg_02.header.seq = 0
        ci_msg_02.header.stamp = rospy.Time(0)
        ci_msg_02.header.frame_id = "kitti360_cam_02"

        ci_msg_02.width = 1400
        ci_msg_02.height = 1400

        ci_msg_02.distortion_model = "plumb_bob"
        ci_msg_02.K = [[
            1.3363220825849971e+03, 0.000000, 7.1694323510126321e+02
        ], [0.000000, 1.3357883350012958e+03, 7.0576498308221585e+02],
                       [0.000000, 0.000000, 0.000000]]

        # NOTE: setting k3 = 0 because is was not supplied in calibrations/image_02.yaml
        ci_msg_02.D = [
            1.6798235660113681e-02, 1.6548773243373522e+00,
            4.2223943394772046e-04, 4.2462134260997584e-04, 0
        ]

        self.ros_publisher_camera_intrinsics_fisheye_left.publish(ci_msg_02)

        # CAM 03 (Fisheye Right)
        ci_msg_03 = CameraInfo()
        ci_msg_03.header.seq = 0
        ci_msg_03.header.stamp = rospy.Time(0)
        ci_msg_03.header.frame_id = "kitti360_cam_03"

        ci_msg_03.width = 1400
        ci_msg_03.height = 1400

        ci_msg_03.distortion_model = "plumb_bob"
        ci_msg_03.K = [[
            1.4854388981875156e+03, 0.000000, 6.9888316784030962e+02
        ], [0.000000, 1.4849477411748708e+03, 6.9814541887723055e+02],
                       [0.000000, 0.000000, 0.000000]]

        # NOTE: setting k3 = 0 because is was not supplied in calibrations/image_03.yaml
        ci_msg_03.D = [
            4.9370396274089505e-02, 4.5068455478645308e+00,
            1.3477698472982495e-03, -7.0340482615055284e-04, 0
        ]

        self.ros_publisher_camera_intrinsics_fisheye_right.publish(ci_msg_03)

    def init_publishers(self):
        self.ros_publisher_clock = rospy.Publisher("clock",
                                                   Clock,
                                                   queue_size=10)
        if self.publish_velodyne:
            self.ros_publisher_3d_raw_velodyne = rospy.Publisher(
                "kitti360/cloud", PointCloud2, queue_size=1)
        if self.publish_sick_points:
            self.ros_publisher_3d_raw_sick_points = rospy.Publisher(
                "kitti360/sick_points", PointCloud2, queue_size=1)
        if self.publish_perspective_rectified_left:
            self.ros_publisher_2d_raw_perspective_rectified_left = rospy.Publisher(
                "kitti360/2d/perspective/rectified_left", Image, queue_size=1)
        if self.publish_perspective_rectified_right:
            self.ros_publisher_2d_raw_perspective_rectified_right = rospy.Publisher(
                "kitti360/2d/perspective/rectified_right", Image, queue_size=1)
        if self.publish_perspective_unrectified_left:
            self.ros_publisher_2d_raw_perspective_unrectified_left = rospy.Publisher(
                "kitti360/2d/perspective/unrectified_left",
                Image,
                queue_size=1)
        if self.publish_perspective_unrectified_right:
            self.ros_publisher_2d_raw_perspective_unrectified_right = rospy.Publisher(
                "kitti360/2d/perspective/unrectified_right",
                Image,
                queue_size=1)
        if self.publish_fisheye_left:
            self.ros_publisher_2d_raw_fisheye_left = rospy.Publisher(
                "kitti360/2d/fisheye/left", Image, queue_size=1)
        if self.publish_fisheye_right:
            self.ros_publisher_2d_raw_fisheye_right = rospy.Publisher(
                "kitti360/2d/fisheye/right", Image, queue_size=1)
        if self.publish_camera_intrinsics:
            self.ros_publisher_camera_intrinsics_unrectified_left = rospy.Publisher(
                "kitti360/2d/perspective/unrectified_left_camera_info",
                CameraInfo,
                queue_size=1)
            self.ros_publisher_camera_intrinsics_unrectified_right = rospy.Publisher(
                "kitti360/2d/perspective/unrectified_right_camera_info",
                CameraInfo,
                queue_size=1)
            self.ros_publisher_camera_intrinsics_fisheye_left = rospy.Publisher(
                "kitti360/2d/fisheye/unrectified_left_camera_info",
                CameraInfo,
                queue_size=1)
            self.ros_publisher_camera_intrinsics_fisheye_right = rospy.Publisher(
                "kitti360/2d/fisheye/unrectified_right_camera_info",
                CameraInfo,
                queue_size=1)
        if self.publish_bounding_boxes:
            self.ros_publisher_bounding_boxes = rospy.Publisher(
                "kitti360/3d/bounding_boxes",
                Kitti360BoundingBox,
                queue_size=1)
        if self.publish_bounding_boxes_rviz_marker:
            self.ros_publisher_bounding_boxes_rviz_marker = rospy.Publisher(
                "kitti360/3d/bounding_boxes_rviz_marker",
                MarkerArray,
                queue_size=1)
        if self.publish_semantics_semantic_left:
            self.ros_publisher_2d_semantics_semantic_left = rospy.Publisher(
                "kitti360/2d/semantics/semantic_left",
                Kitti360SemanticID,
                queue_size=1)
        if self.publish_semantics_semantic_right:
            self.ros_publisher_2d_semantics_semantic_right = rospy.Publisher(
                "kitti360/2d/semantics/semantic_right",
                Kitti360SemanticID,
                queue_size=1)
        if self.publish_semantics_semantic_rgb_left:
            self.ros_publisher_2d_semantics_semantic_rgb_left = rospy.Publisher(
                "kitti360/2d/semantics/semantic_rgb_left",
                Kitti360SemanticRGB,
                queue_size=1)
        if self.publish_semantics_semantic_rgb_right:
            self.ros_publisher_2d_semantics_semantic_rgb_right = rospy.Publisher(
                "kitti360/2d/semantics/semantic_rgb_right",
                Kitti360SemanticRGB,
                queue_size=1)
        if self.publish_semantics_instance_left:
            self.ros_publisher_2d_semantics_instance_left = rospy.Publisher(
                "kitti360/2d/semantics/instance_left",
                Kitti360InstanceID,
                queue_size=1)
        if self.publish_semantics_instance_right:
            self.ros_publisher_2d_semantics_instance_right = rospy.Publisher(
                "kitti360/2d/semantics/instance_right",
                Kitti360InstanceID,
                queue_size=1)
        if self.publish_semantics_confidence_left:
            self.ros_publisher_2d_semantics_confidence_left = rospy.Publisher(
                "kitti360/2d/semantics/confidence_left",
                Kitti360Confidence,
                queue_size=1)
        if self.publish_semantics_confidence_right:
            self.ros_publisher_2d_semantics_confidence_right = rospy.Publisher(
                "kitti360/2d/semantics/confidence_right",
                Kitti360Confidence,
                queue_size=1)
        if self.publish_3d_semantics_static:
            self.ros_publisher_3d_semantics_static = rospy.Publisher(
                "kitti360/3d/semantics/static", PointCloud2, queue_size=1)
        if self.publish_3d_semantics_dynamic:
            self.ros_publisher_3d_semantics_dynamic = rospy.Publisher(
                "kitti360/3d/semantics/dynamic", PointCloud2, queue_size=1)

    def read_bounding_boxes(self):
        if not self.publish_bounding_boxes and not self.publish_bounding_boxes_rviz_marker:
            return

        path = os.path.join(
            self.DATA_DIRECTORY, "data_3d_bboxes/train_full",
            f"2013_05_28_drive_{int(self.SEQUENCE):04d}_sync.xml")

        if not os.path.exists(path):
            self.publish_bounding_boxes = False
            self.publish_bounding_boxes_rviz_marker = False
            return

        temp = xmltodict.parse(open(path).read())["opencv_storage"]

        self.bounding_box_data = dict(zip(range(len(temp)), temp.values()))

        df = pd.DataFrame.from_dict(self.bounding_box_data, orient="index")[[
            "start_frame", "end_frame"
        ]]
        df["index"] = list(range(df.shape[0]))
        df["start_frame"] = df["start_frame"].astype(int)
        df["end_frame"] = df["end_frame"].astype(int)
        df = df.groupby(by=["start_frame", "end_frame"])["index"].apply(
            tuple).reset_index()
        df.columns = ["start_frame", "end_frame", "indices"]

        self.bounding_box_frame_ranges = df

    def read_timestamps(self):
        # this is in all cases the first sick timestamps because that timestamp
        # is always about half a second earlier than the first velodyne
        # timestamp
        begin_timestamp_sequence = {
            "0000": pd.Timestamp("2013-05-28 08:46:02.482785814"),
            "0002": pd.Timestamp("2013-05-28 09:23:32.515737801"),
            "0003": pd.Timestamp("2013-05-28 09:57:31.550356159"),
            "0004": pd.Timestamp("2013-05-28 10:00:58.733963786"),
            "0005": pd.Timestamp("2013-05-28 10:24:13.567881542"),
            "0006": pd.Timestamp("2013-05-28 10:37:26.212659328"),
            "0007": pd.Timestamp("2013-05-28 10:56:16.372326181"),
            "0009": pd.Timestamp("2013-05-28 11:36:55.283363208"),
            "0010": pd.Timestamp("2013-05-28 12:01:27.300138700")
        }

        def _read_timestamps(path):
            # timestamps.txt contains real-world dates
            # everything was recorded after 2013-05-28 8:46
            # resulting timestamps is the time that has passed from the
            # beginning of the in nanosecond precision

            return (
                pd.to_datetime(
                    pd.read_csv(path, header=None).squeeze("columns")) -
                begin_timestamp_sequence[self.SEQUENCE]
            ).apply(lambda t: rospy.Time(
                secs=t.seconds, nsecs=(t.microseconds * 1000) + t.nanoseconds))

        # VELODYNE POINTS
        # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/velodyne_points/timestamps.txt
        try:
            self.timestamps_velodyne = _read_timestamps(
                os.path.join(self.DATA_DIRECTORY, "data_3d_raw",
                             self.SEQUENCE_DIRECTORY,
                             "velodyne_points/timestamps.txt"))
            self.total_number_of_frames_velodyne = self.timestamps_velodyne.shape[
                0]
            self.total_simulation_time = self.timestamps_velodyne.iloc[
                -1].to_sec()
        except FileNotFoundError:
            rospy.logerr("timestamps for velodyne not found. FATAL")
            rospy.signal_shutdown(
                "cannot find velodyne timestamps --> need for execution")
            exit()

        # SICK POINTS
        # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/sick_points/timestamps.txt
        try:
            self.timestamps_sick_points = _read_timestamps(
                os.path.join(self.DATA_DIRECTORY, "data_3d_raw",
                             self.SEQUENCE_DIRECTORY,
                             "sick_points/timestamps.txt"))
            self.total_number_of_frames_sick = self.timestamps_sick_points.shape[
                0]
            if self.timestamps_sick_points.iloc[-1].to_sec(
            ) > self.total_simulation_time:
                self.total_simulation_time = self.timestamps_sick_points.iloc[
                    -1].to_sec()
        except FileNotFoundError:
            rospy.logerr("timestamps for sick points not found. Disabling.")
            self.publish_sick_points = False

        # PERSPECTIVE CAMERA (00/01)
        # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{00|01}/timestamps.txt
        try:
            self.timestamps_perspective_camera_00 = _read_timestamps(
                os.path.join(self.DATA_DIRECTORY, "data_2d_raw",
                             self.SEQUENCE_DIRECTORY,
                             "image_00/timestamps.txt"))
        except FileNotFoundError:
            rospy.logerr(
                "timestamps for left perspective camera not found. Disabling.")
            self.publish_perspective_rectified_left = False
            self.publish_perspective_unrectified_left = False
        try:
            self.timestamps_perspective_camera_01 = _read_timestamps(
                os.path.join(self.DATA_DIRECTORY, "data_2d_raw",
                             self.SEQUENCE_DIRECTORY,
                             "image_01/timestamps.txt"))
        except FileNotFoundError:
            rospy.logerr(
                "timestamps for right perspective camera not found. Disabling."
            )
            self.publish_perspective_rectified_right = False
            self.publish_perspective_unrectified_right = False

        # FISHEYE CAMERA (02/03)
        # data_2d_raw/2013_05_28_drive_{seq:0>4}_sync/image_{02|03}/timestamps.txt
        try:
            self.timestamps_fisheye_camera_02 = _read_timestamps(
                os.path.join(self.DATA_DIRECTORY, "data_2d_raw",
                             self.SEQUENCE_DIRECTORY,
                             "image_02/timestamps.txt"))
        except FileNotFoundError:
            rospy.logerr(
                "timestamps for left fisheye camera not found. Disabling.")
            self.publish_fisheye_left = False

        try:
            self.timestamps_fisheye_camera_03 = _read_timestamps(
                os.path.join(self.DATA_DIRECTORY, "data_2d_raw",
                             self.SEQUENCE_DIRECTORY,
                             "image_03/timestamps.txt"))
        except FileNotFoundError:
            rospy.logerr(
                "timestamps for right fisheye camera not found. Disabling.")
            self.publish_fisheye_right = False

        return 0

    def read_poses(self):
        data_path = os.path.join(self.DATA_DIRECTORY, "data_poses",
                                 self.SEQUENCE_DIRECTORY, "poses.txt")

        if not os.path.exists(data_path):
            rospy.logerr(
                f"Could not find poses.txt. File does not exist or is not in correct location (expected: {data_path}). FATAL"
            )
            rospy.signal_shutdown("poses.txt does not exist but is required")
            exit()

        poses = pd.read_csv(data_path, sep=" ", header=None)
        # index = frame id = (was first column)
        poses = poses.set_index(0)
        poses = poses.apply(lambda row: np.array(row).reshape(3, 4), axis=1)

        # pandas Series index is frame_id
        # --> access using .loc[frame_id] (may result in KeyError because not all poses exist)
        self.poses = poses

    def read_3d_semantics_dir(self):

        def _read_dir(p):
            # extract start and end frame from
            df = pd.Series(os.listdir(p)).to_frame(name="filename")
            df["start_frame"], df["end_frame"] = zip(*df["filename"].str.split(
                '[_.]').str[:2].apply(lambda x: (int(x[0]), int(x[1]))))
            # check if the files actually contain
            df["filter"] = df["filename"].apply(
                lambda fn: open(os.path.join(p, fn), encoding="ISO-8859-1"
                                ).readlines()[3] != "element vertex 0\n")
            df = df[df["filter"]]
            df = df.sort_values(by="start_frame")
            return df

        if self.publish_3d_semantics_static:
            folder_path = os.path.join(self.DATA_DIRECTORY,
                                       "data_3d_semantics/train",
                                       self.SEQUENCE_DIRECTORY, "static")
            if os.path.exists(folder_path):
                self.bounds_3d_sem_static_index = _read_dir(folder_path)
            else:
                rospy.logerr(
                    "Directory does not exist. Disabling 3d semantics static")
                self.publish_3d_semantics_static = False

        if self.publish_3d_semantics_dynamic:
            folder_path = os.path.join(self.DATA_DIRECTORY,
                                       "data_3d_semantics/train",
                                       self.SEQUENCE_DIRECTORY, "dynamic")
            if os.path.exists(folder_path):
                self.bounds_3d_sem_dynamic_index = _read_dir(folder_path)
            else:
                rospy.logerr(
                    "Directory does not exist. Disabling 3d semantics dynamic")
                self.publish_3d_semantics_dynamic = False

    # ------------------------------------------
    # PUBLISHING
    def handle_sick_points_publishing(self):
        """handles publishing of sick points, which have a higher refresh rate
        (and therefore more frames) to be published"""

        s = time.time()

        if not self.publish_sick_points:
            return dict()

        next_frame = self.timestamps_sick_points.searchsorted(
            self.sim_clock.clock) - 1

        # if we are before first frame or or frame that needs to be published
        # has already been published --> return
        if next_frame == -1 or next_frame == self.last_published_sick_frame:
            return dict()

        # check if and how many frames we are skipping
        if self.last_published_sick_frame is not None:
            skipped = next_frame - self.last_published_sick_frame - 1
            logfunc = rospy.logwarn if skipped > 0 else rospy.loginfo
            if skipped >= 0:
                skipped_string = f"(skipping {skipped})"
            else:
                skipped_string = "(backwards)"
            logfunc(
                f"new SICK frame" + f" {skipped_string:<18}" +
                f"{self._convert_frame_int_to_string(self.last_published_sick_frame)}"
                + f" -> {self._convert_frame_int_to_string(next_frame)} " +
                f"({self.sim_clock.clock.to_sec():.2f}s, {((self.sim_clock.clock.to_sec()/self.total_simulation_time)*100):.1f}%)"
            )

        # --------------------------------------------------
        # construct and publish sick points message
        # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/sick_points/data/{frame:0>10}.bin
        data_path = os.path.join(self.DATA_DIRECTORY, "data_3d_raw",
                                 self.SEQUENCE_DIRECTORY, "sick_points",
                                 "data")

        if not os.path.exists(data_path):
            rospy.logerr(f"{data_path} does not exist. Disabling sick points.")
            self.publish_sick_points = False
            return dict()

        pointcloud_bin = np.fromfile(os.path.join(
            data_path,
            self._convert_frame_int_to_string(next_frame) + ".bin"),
                                     dtype=np.float32)
        # copied from:
        # https://github.com/autonomousvision/kitti360Scripts/blob/7ecc14eab6fa30e5d2ac71ad37fed2bb4b0b8073/kitti360scripts/viewer/kitti360Viewer3DRaw.py#L60
        pointcloud_bin = pointcloud_bin.reshape((-1, 2))
        pointcloud_bin = np.concatenate([
            np.zeros_like(pointcloud_bin[:, 0:1]), -pointcloud_bin[:, 0:1],
            pointcloud_bin[:, 1:2]
        ],
                                        axis=1)

        # PointCloud2 Message http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html
        # Header http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Header.html
        # PointField http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointField.html
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.timestamps_sick_points.iloc[next_frame]
        cloud_msg.header.frame_id = "kitti360_sick_points"
        cloud_msg.header.seq = next_frame

        # body
        cloud_msg.height = pointcloud_bin.shape[0]
        cloud_msg.width = 1
        cloud_msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        # both True and False worked, so idk
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4bytes (float32)
        cloud_msg.row_step = 12  # a row is a point in our case
        cloud_msg.data = pointcloud_bin.tobytes()
        cloud_msg.is_dense = True

        # publish
        self.ros_publisher_3d_raw_sick_points.publish(cloud_msg)
        # --------------------------------------------------

        # save what has been published
        self.last_published_sick_frame = next_frame

        return dict([("sick", time.time() - s)])

    def _publish_bounding_boxes(self, frame):
        if not self.publish_bounding_boxes:
            return dict()

        # for benchmarking
        s = time.time()

        # NOTE ranges overlap a little bit ~15 frames
        # this code publishes the latest possible pointcloud (if there are two)
        index = self.bounding_box_frame_ranges["start_frame"].searchsorted(
            frame)
        if index == self.previous_published_index_bb:
            return dict([("bounding boxes rviz marker", time.time() - s)])
        else:
            self.previous_published_index_bb = index
        bb_indices = self.bounding_box_frame_ranges["indices"].iloc[index - 1]

        def _get_multiarray(name):
            num_rows = int(bb_data[name]["rows"])
            num_cols = int(bb_data[name]["cols"])

            transform_array_layout = MultiArrayLayout()
            dim0 = MultiArrayDimension()
            dim0.label = "rows"
            dim0.size = num_rows
            dim0.stride = num_rows * num_cols

            dim1 = MultiArrayDimension()
            dim1.label = "cols"
            dim1.size = num_cols
            dim1.stride = num_cols

            transform_array_layout.dim = [dim0, dim1]
            transform_array_layout.data_offset = 0
            if bb_data[name]["dt"] == "u":
                # assuming int16 is enoug
                transform_array = Int16MultiArray()
                np_dtype = np.int16
            elif bb_data[name]["dt"] == "f":
                transform_array = Float32MultiArray()
                np_dtype = np.float32
            else:
                rospy.logerr(
                    f"Unknown datatype: {bb_data[name]['dt']} in bounding box field {name} | {index=} | {frame=}"
                )
                rospy.signal_shutdown("See previous error message")
                exit()

            transform_array.data = np.fromstring(bb_data[name]["data"],
                                                 sep=" ",
                                                 dtype=np_dtype)
            transform_array.layout = transform_array_layout

            return transform_array

        for bb_index in bb_indices:
            bb_data = self.bounding_box_data[bb_index]

            bb = Kitti360BoundingBox()

            # +1 because they start counting at 1 and we start counting at 0
            assert bb_index + 1 == int(
                bb_data["index"]
            ), "bounding box indices unexpectedly do not match"

            # NOTE casting takes time and does not need to be done at runtime.
            # If running time of bounding box publishing is an issue, this could be improved.

            bb.index = int(bb_data["index"])
            bb.label = bb_data["label"]
            bb.category = bb_data["category"]

            # NOTE: no clue what this is
            bb.level_min = float(bb_data["level_min"])
            bb.level_max = float(bb_data["level_max"])

            # NOTE we may need to do something different depending on whether
            # the bounding box is dynamic or not
            bb.dynamic = bool(bb_data["dynamic"])
            bb.dynamicSeq = abs(int(bb_data["dynamicSeq"]))
            bb.dynamicIdx = abs(int(bb_data["dynamicIdx"]))

            bb.transform = _get_multiarray("transform")
            bb.vertices = _get_multiarray("vertices")
            bb.faces = _get_multiarray("faces")

            self.ros_publisher_bounding_boxes.publish(bb)

        return dict([("bounding boxes", time.time() - s)])

    def _publish_bounding_boxes_rviz_marker(self, frame):
        if not self.publish_bounding_boxes_rviz_marker:
            return dict()

        # for benchmarking
        s = time.time()

        # NOTE ranges overlap a little bit ~15 frames
        # this code publishes the latest possible pointcloud (if there are two)
        index = self.bounding_box_frame_ranges["start_frame"].searchsorted(
            frame)
        if index == self.previous_published_index_bb_rviz:
            return dict([("bounding boxes rviz marker", time.time() - s)])
        else:
            self.previous_published_index_bb_rviz = index

        bb_indices = self.bounding_box_frame_ranges["indices"].iloc[index - 1]

        marker_array = []
        for bb_index in bb_indices:
            bb_data = self.bounding_box_data[bb_index]

            # skipt dynamic objects
            if int(bb_data["timestamp"]) > 0:
                continue

            # +1 because they start counting at 1 and we start counting at 0
            assert bb_index + 1 == int(
                bb_data["index"]
            ), "bounding box indices unexpectedly do not match"

            # FROM: opencv storage method:
            # list of points and then list three-tuples of point indices
            # indicating which points form a surface
            # TO: list of points, where each three points form a surface
            # --> take three tuples from opencv format and replace indices with
            # actual points

            # contains coordinates at i of point i in Point.msg format
            vertices_ros_point_format = []
            input_vertices = np.fromstring(
                bb_data["vertices"]["data"], sep=" ",
                dtype=np.float32).reshape(int(bb_data["vertices"]["rows"]),
                                          int(bb_data["vertices"]["cols"]))

            input_transform = np.fromstring(bb_data["transform"]["data"],
                                            sep=" ",
                                            dtype=np.float32).reshape(4, 4)

            input_faces = np.fromstring(bb_data["faces"]["data"],
                                        sep=" ",
                                        dtype=np.int32)

            input_transform_slided = input_transform[:3, :3]
            for vertex in input_vertices:
                # first apply transform
                v = np.matmul(input_transform_slided, vertex)

                p = Point()
                p.x = v[0]
                p.y = v[1]
                p.z = v[2]
                vertices_ros_point_format.append(p)

            # translate vertex index to actual point
            points_rviz_format = []
            for vertex_index in input_faces:
                points_rviz_format.append(
                    vertices_ros_point_format[vertex_index])

            # create message
            marker_msg = Marker()

            marker_msg.header.seq = bb_index
            marker_msg.header.stamp = self.timestamps_velodyne.iloc[frame]
            marker_msg.header.frame_id = "map"

            # namespace: not sure if this is correct
            marker_msg.ns = str(frame)
            marker_msg.id = bb_index

            marker_msg.type = marker_msg.TRIANGLE_LIST
            marker_msg.action = marker_msg.ADD

            marker_msg.pose.position.x = input_transform[0, 3]
            marker_msg.pose.position.y = input_transform[1, 3]
            marker_msg.pose.position.z = input_transform[2, 3]
            # identity quaternion -> we apply the transformation already
            # directly to the points
            marker_msg.pose.orientation.x = 0
            marker_msg.pose.orientation.y = 0
            marker_msg.pose.orientation.z = 0
            marker_msg.pose.orientation.w = 1

            # --> original size
            marker_msg.scale.x = 1
            marker_msg.scale.y = 1
            marker_msg.scale.z = 1

            # TODO maybe make this configurable or choose some better dynamic (?) number
            # right now --> just keeping it there "forever"
            marker_msg.lifetime = rospy.Duration(secs=100000)

            # TODO not sure
            marker_msg.frame_locked = True

            marker_msg.points = points_rviz_format

            # not transparent (is also not supported by the RVIZ)
            marker_msg.color.a = 1
            c = ColorRGBA()
            c.a = 1

            # some bounding box labels are not present in the labels.py file
            # provided by the KITTI-360 authors. Just for colors, we are
            # mapping them to different labels.
            missing_label_mapping = {
                "unknownGround": "ground",
                "unknownConstruction": "unknown construction",
                "unknownVehicle": "unknown vehicle",
                "unknownObject": "unknown object",
                "bigPole": "pole",
                "driveway": "road",
                "railtrack": "rail track",
                "trafficSign": "traffic sign",
                "trashbin": "trash bin",
                "pedestrian": "person",
                "guardrail": "guard rail",
                "smallPole": "smallpole",
                "trafficLight": "traffic light",
                "vendingmachine": "vending machine"
            }
            if bb_data["label"] not in name2label:
                c.r = name2label[missing_label_mapping[
                    bb_data["label"]]].color[0]
                c.g = name2label[missing_label_mapping[
                    bb_data["label"]]].color[1]
                c.b = name2label[missing_label_mapping[
                    bb_data["label"]]].color[2]
            else:
                c.r = name2label[bb_data["label"]].color[0]
                c.g = name2label[bb_data["label"]].color[1]
                c.b = name2label[bb_data["label"]].color[2]
            marker_msg.colors = [c] * len(points_rviz_format)

            marker_array.append(marker_msg)

        marker_array_msg = MarkerArray()
        marker_array_msg.markers = marker_array
        self.ros_publisher_bounding_boxes_rviz_marker.publish(marker_array_msg)

        return dict([("bounding boxes rviz marker", time.time() - s)])

    def _publish_images(self, frame):

        def _construct_message(timestamp_ser, image_dir, frame_id, frame_index,
                               width, height):
            image_msg = Image()

            assert timestamp_ser is not None, "timestamp series cannot be None at the point"

            # not sure what we want here
            image_msg.is_bigendian = False
            image_msg.header.stamp = timestamp_ser.iloc[frame_index]
            image_msg.header.frame_id = frame_id
            image_msg.header.seq = frame

            image_msg.width = width
            image_msg.height = height

            # 8-bit/color RGB --> rgb8
            image_msg.encoding = "rgb8"

            # each px contains three uint8
            image_msg.step = 3 * image_msg.width

            try:
                image = imageio.v3.imread(
                    os.path.join(
                        self.DATA_DIRECTORY, "data_2d_raw",
                        self.SEQUENCE_DIRECTORY, image_dir,
                        self._convert_frame_int_to_string(frame) + ".png"))
                image_msg.data = image.tobytes()
            except FileNotFoundError:
                return None

            return image_msg

        # for benchmarking
        durations = dict()
        s = time.time()
        if self.publish_perspective_rectified_left:
            if (msg :=
                    _construct_message(self.timestamps_perspective_camera_00,
                                       "image_00/data_rect",
                                       "kitti360_cam_00",
                                       frame,
                                       width=1408,
                                       height=376)) is not None:
                self.ros_publisher_2d_raw_perspective_rectified_left.publish(
                    msg)
                durations["image_00_data_rect"] = time.time() - s
            elif self.publish_perspective_rectified_left:
                self.publish_perspective_rectified_left = False
                rospy.logerr(
                    f"Directory does not exist. Disabling camera perspective rectified left."
                )

        s = time.time()
        if self.publish_perspective_rectified_right:
            if (msg :=
                    _construct_message(self.timestamps_perspective_camera_01,
                                       "image_01/data_rect",
                                       "kitti360_cam_01",
                                       frame,
                                       width=1408,
                                       height=376)) is not None:
                self.ros_publisher_2d_raw_perspective_rectified_right.publish(
                    msg)
                durations["image_01_data_rect"] = time.time() - s
            elif self.publish_perspective_rectified_right:
                self.publish_perspective_rectified_right = False
                rospy.logerr(
                    f"Directory does not exist. Disabling camera perspective rectified right."
                )

        s = time.time()
        if self.publish_perspective_unrectified_left:
            if (msg :=
                    _construct_message(self.timestamps_perspective_camera_00,
                                       "image_00/data_rgb",
                                       "kitti360_cam_00",
                                       frame,
                                       width=1392,
                                       height=512)) is not None:
                self.ros_publisher_2d_raw_perspective_unrectified_left.publish(
                    msg)
                durations["image_00_data_rgb"] = time.time() - s
            elif self.publish_perspective_unrectified_left:
                self.publish_perspective_unrectified_left = False
                rospy.logerr(
                    f"Directory does not exist. Disabling camera perspective unrectified left."
                )

        s = time.time()
        if self.publish_perspective_unrectified_right:
            if (msg :=
                    _construct_message(self.timestamps_perspective_camera_01,
                                       "image_01/data_rgb",
                                       "kitti360_cam_01",
                                       frame,
                                       width=1392,
                                       height=512)) is not None:
                self.ros_publisher_2d_raw_perspective_unrectified_right.publish(
                    msg)
                durations["image_01_data_rgb"] = time.time() - s
            elif self.publish_perspective_unrectified_right:
                self.publish_perspective_unrectified_right = False
                rospy.logerr(
                    f"Directory does not exist. Disabling camera perspective unrectified right."
                )

        s = time.time()
        if self.publish_fisheye_left:
            if (msg := _construct_message(self.timestamps_fisheye_camera_02,
                                          "image_02/data_rgb",
                                          "kitti360_cam_02",
                                          frame,
                                          width=1400,
                                          height=1400)) is not None:
                self.ros_publisher_2d_raw_fisheye_left.publish(msg)
                durations["image_02_data_rgb"] = time.time() - s
            elif self.publish_fisheye_left:
                self.publish_fisheye_left = False
                rospy.logerr(
                    f"Directory does not exist. Disabling camera fisheye left."
                )

        s = time.time()
        if self.publish_fisheye_right:
            if (msg := _construct_message(self.timestamps_fisheye_camera_03,
                                          "image_03/data_rgb",
                                          "kitti360_cam_03",
                                          frame,
                                          width=1400,
                                          height=1400)) is not None:
                self.ros_publisher_2d_raw_fisheye_right.publish(msg)
                durations["image_03_data_rgb"] = time.time() - s
            elif self.publish_fisheye_right:
                self.publish_fisheye_right = False
                rospy.logerr(
                    f"Directory does not exist. Disabling camera fisheye right."
                )

        return durations

    def _publish_velodyne(self, frame):
        # if this is false either it was set or the data dir was not found in a
        # previous attempt of this function
        if not self.publish_velodyne:
            return dict()

        # for benchmarking
        s = time.time()

        # data_3d_raw/2013_05_28_drive_{seq:0>4}_sync/velodyne_points/data/{frame:0>10}.bin
        data_path = os.path.join(self.DATA_DIRECTORY, "data_3d_raw",
                                 self.SEQUENCE_DIRECTORY, "velodyne_points",
                                 "data")

        if not os.path.exists(data_path):
            rospy.logerr(
                f"{data_path} does not exist. Disabling velodyne pointclouds.")
            self.publish_velodyne = False
            return dict()

        pointcloud_bin = np.fromfile(os.path.join(
            data_path,
            self._convert_frame_int_to_string(frame) + ".bin"),
                                     dtype=np.float32)
        pointcloud_bin = pointcloud_bin.reshape((-1, 4))

        # PointCloud2 Message http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html
        # Header http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Header.html
        # PointField http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointField.html
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.timestamps_velodyne.iloc[frame]
        cloud_msg.header.frame_id = "kitti360_velodyne"
        cloud_msg.header.seq = frame

        # body
        cloud_msg.height = pointcloud_bin.shape[0]
        cloud_msg.width = 1
        cloud_msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1)
        ]
        # both True and False worked, so idk
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 * 4bytes (float32)
        cloud_msg.row_step = 16  # a row is a point in our case
        cloud_msg.data = pointcloud_bin.tobytes()
        cloud_msg.is_dense = True

        # publish
        self.ros_publisher_3d_raw_velodyne.publish(cloud_msg)

        return dict([("velodyne", time.time() - s)])

    def _publish_transforms(self, frame):
        # FIXME the pointcloud sometimes jumps out and back when playing at
        # full speed in RVIZ

        # transform from GPS/IMU to map (which is identical to world)
        transform_broadcaster = tf2_ros.TransformBroadcaster()

        t = TransformStamped()
        # this needs to be the timestamp that the pointcloud also uses
        t.header.stamp = self.timestamps_velodyne.iloc[frame]
        t.header.frame_id = "map"
        t.child_frame_id = "kitti360_gpsimu"

        # There is never a transform for frame 0. Authors said in email that
        # pose estimation starts only from frame 1, because of how they do GPS
        # measurements. Using pose from frame 1 for frame 0 as the position has
        # likely not changed between the first two frames all that much and we
        # need some pose for frame 0.
        if frame == 0:
            frame = 1

        # if pos does not exist --> select one from earlier
        # (pos not existent == change between poses was minimimal / below threshold)
        # if it does exist select matching one (<=)
        tf_matrix = self.poses.loc[self.poses.index[self.poses.index <= frame]
                                   [-1]]

        # last column in 3x4 matrix is translation vector
        t.transform.translation = Vector3(x=tf_matrix[0, 3],
                                          y=tf_matrix[1, 3],
                                          z=tf_matrix[2, 3])

        # convert from rotation matrix to quatertion
        # (first three columns are transformation matrix)
        tf_matrix44 = np.vstack((tf_matrix, [0, 0, 0, 1]))
        quat = transformations.quaternion_from_matrix(tf_matrix44)
        t.transform.rotation = Quaternion(x=quat[0],
                                          y=quat[1],
                                          z=quat[2],
                                          w=quat[3])

        transform_broadcaster.sendTransform(t)

        return dict()

    def _publish_2d_semantics(self, frame):
        durations = dict()

        # NOTE the are gaps and the dataset does not contain semantics for all
        # images. That is why we simply pass when the file it not found

        def _pub(subdir, pub_bool, dtype, publisher, desc):
            nonlocal durations

            s = time.time()
            temp_dir = os.path.join(self.DATA_DIRECTORY,
                                    "data_2d_semantics/train",
                                    self.SEQUENCE_DIRECTORY, subdir)
            if pub_bool and (not os.path.exists(temp_dir)):
                rospy.logerr(f"Directory does not exist. Disabling {desc}.")
                return False
            if pub_bool:
                try:
                    data = imageio.v3.imread(
                        os.path.join(temp_dir, f"{frame:010d}.png")).tobytes()
                    semantic = dtype()
                    semantic.width = 1408
                    semantic.height = 376
                    semantic.data = data
                    publisher.publish(semantic)
                except FileNotFoundError:
                    pass

                durations[desc] = time.time() - s
                return True

        self.publish_semantics_semantic_left = _pub(
            "image_00/semantic/", self.publish_semantics_semantic_left,
            Kitti360SemanticID, self.ros_publisher_2d_semantics_semantic_left,
            "2d semanticID left")
        self.publish_semantics_semantic_right = _pub(
            "image_01/semantic/", self.publish_semantics_semantic_right,
            Kitti360SemanticID, self.ros_publisher_2d_semantics_semantic_right,
            "2d semanticID right")
        self.publish_semantics_semantic_rgb_left = _pub(
            "image_00/semantic_rgb/", self.publish_semantics_semantic_rgb_left,
            Kitti360SemanticRGB,
            self.ros_publisher_2d_semantics_semantic_rgb_left,
            "2d semantic rgb left")
        self.publish_semantics_semantic_rgb_right = _pub(
            "image_01/semantic_rgb/",
            self.publish_semantics_semantic_rgb_right, Kitti360SemanticRGB,
            self.ros_publisher_2d_semantics_semantic_rgb_right,
            "2d semantic rgb right")
        self.publish_semantics_instance_left = _pub(
            "image_00/instance/", self.publish_semantics_instance_left,
            Kitti360InstanceID, self.ros_publisher_2d_semantics_instance_left,
            "2d instanceID left")
        self.publish_semantics_instance_right = _pub(
            "image_01/instance/", self.publish_semantics_instance_right,
            Kitti360InstanceID, self.ros_publisher_2d_semantics_instance_right,
            "2d instanceID right")
        self.publish_semantics_confidence_left = _pub(
            "image_00/confidence/", self.publish_semantics_confidence_left,
            Kitti360Confidence,
            self.ros_publisher_2d_semantics_confidence_left,
            "2d confidence left")
        self.publish_semantics_confidence_right = _pub(
            "image_01/confidence/", self.publish_semantics_confidence_right,
            Kitti360Confidence,
            self.ros_publisher_2d_semantics_confidence_right,
            "2d confidence right")

        return durations

    def _publish_3d_semantics(self, frame):
        # "annotations are provided on accumulated point clouds defined in a
        # world coordinate system"
        #
        # if the directory does not exists the bools are already set to False
        # in self.read_3d_semantics_dir()

        durations = dict()
        if self.publish_3d_semantics_dynamic:
            s = time.time()

            # NOTE ranges overlap a little bit ~15 frames
            # this code publishes the latest possible pointcloud (if there are two)
            cand = self.bounds_3d_sem_dynamic_index[
                (self.bounds_3d_sem_dynamic_index["start_frame"] <= frame)
                & (self.bounds_3d_sem_dynamic_index["end_frame"] > frame)]
            if not cand.empty:
                path = os.path.join(self.DATA_DIRECTORY,
                                    "data_3d_semantics/train",
                                    self.SEQUENCE_DIRECTORY, "dynamic",
                                    cand["filename"].iloc[0])
                if not self.filename_3d_semantics_dynamic == path:
                    df = PyntCloud.from_file(path).points
                    self.records_3d_semantics_dynamic = df.to_records(
                        index=False).tobytes()
                    self.filename_3d_semantics_dynamic = path
                    rospy.loginfo(
                        f"loaded new batch of DYNAMIC 3d semantics: frames {cand['start_frame'].iloc[0]} to {cand['end_frame'].iloc[0]}"
                    )

                msg = PointCloud2()
                msg.header.stamp = self.timestamps_velodyne.iloc[frame]
                msg.header.frame_id = "map"  # == "world"
                msg.header.seq = frame

                # body
                row_byte_length = 32
                msg.height = abs(
                    int(
                        len(self.records_3d_semantics_dynamic) /
                        row_byte_length))
                msg.width = 1
                msg.fields = [
                    PointField("x", 0, PointField.FLOAT32, 1),
                    PointField("y", 4, PointField.FLOAT32, 1),
                    PointField("z", 8, PointField.FLOAT32, 1),
                    PointField("red", 12, PointField.UINT8, 1),
                    PointField("green", 13, PointField.UINT8, 1),
                    PointField("blue", 14, PointField.UINT8, 1),
                    PointField("semantic", 15, PointField.INT32, 1),
                    PointField("instance", 19, PointField.INT32, 1),
                    PointField("visible", 23, PointField.UINT8, 1),
                    PointField("timestamp", 24, PointField.INT32, 1),
                    PointField("confidence", 28, PointField.FLOAT32, 1),
                ]
                # both True and False worked, so idk
                msg.is_bigendian = False
                msg.point_step = row_byte_length  #
                msg.row_step = row_byte_length  # a row is a point in our case
                msg.data = self.records_3d_semantics_dynamic
                msg.is_dense = True
                self.ros_publisher_3d_semantics_dynamic.publish(msg)
                durations["3d semantics dynamic"] = time.time() - s

        if self.publish_3d_semantics_static:
            s = time.time()
            # NOTE ranges overlap a little bit ~15 frames
            # this code publishes the latest possible pointcloud (if there are two)
            cand = self.bounds_3d_sem_static_index[
                (self.bounds_3d_sem_static_index["start_frame"] <= frame)
                & (self.bounds_3d_sem_static_index["end_frame"] > frame)]
            if not cand.empty:
                path = os.path.join(self.DATA_DIRECTORY,
                                    "data_3d_semantics/train",
                                    self.SEQUENCE_DIRECTORY, "static",
                                    cand["filename"].iloc[0])
                if not self.filename_3d_semantics_static == path:
                    df = PyntCloud.from_file(path).points
                    self.records_3d_semantics_static = df.to_records(
                        index=False).tobytes()
                    self.filename_3d_semantics_static = path
                    rospy.loginfo(
                        f"loaded new batch of STATIC 3d semantics: frames {cand['start_frame'].iloc[0]} to {cand['end_frame'].iloc[0]}"
                    )

                msg = PointCloud2()
                msg.header.stamp = self.timestamps_velodyne.iloc[frame]
                msg.header.frame_id = "map"  # == "world"
                msg.header.seq = frame

                # body
                row_byte_length = 28
                msg.height = abs(
                    int(
                        len(self.records_3d_semantics_static) /
                        row_byte_length))
                msg.width = 1
                msg.fields = [
                    PointField("x", 0, PointField.FLOAT32, 1),
                    PointField("y", 4, PointField.FLOAT32, 1),
                    PointField("z", 8, PointField.FLOAT32, 1),
                    PointField("red", 12, PointField.UINT8, 1),
                    PointField("green", 13, PointField.UINT8, 1),
                    PointField("blue", 14, PointField.UINT8, 1),
                    PointField("semantic", 15, PointField.INT32, 1),
                    PointField("instance", 19, PointField.INT32, 1),
                    PointField("visible", 23, PointField.UINT8, 1),
                    PointField("confidence", 24, PointField.FLOAT32, 1),
                ]
                # both True and False worked, so idk
                msg.is_bigendian = False
                msg.point_step = row_byte_length  #
                msg.row_step = row_byte_length  # a row is a point in our case
                msg.data = self.records_3d_semantics_static
                msg.is_dense = True
                self.ros_publisher_3d_semantics_static.publish(msg)
                durations["3d semantics static"] = time.time() - s

        return durations

    # ------------------------------------------
    # COMMAND LINE INTERFACE
    def print_help(self):
        desc = {
            "*": "print this",
            "s": "step to next VELODYNE frame (skips 3 SICK frames)",
            "S": "step to previous VELODYNE frame  (skips 3 SICK frames)",
            "d": "step to next SICK frame",
            "D": "step to previous SICK frame",
            "<space>": "pause/unpause simulation",
            "k": "increase playback speed factor by 0.1",
            "j": "decrease playback speed factor by 0.1",
            "[0-9]": "seek to x0% of simulation (e.g. 6 -> 60%)",
            "b": "print duration of each publishing step",
        }

        # determine width of both columns
        margin = 2
        key_col_width = max(map(len, desc.keys()))
        desc_col_width = max(map(len, desc.values()))
        # +3 for colon and |
        # 4*margin because we apply margin before and each of # the two colunms
        total_width = (key_col_width + desc_col_width + 3 + margin * 4)

        end_line = "+" + "-" * (total_width - 2) + "+"
        rospy.loginfo(end_line)
        rospy.loginfo("|  KEY MAPPINGS".ljust(total_width - 1) + "|")

        for key, desc in desc.items():
            s = "|"
            s += ' ' * margin
            s += key.ljust(key_col_width)
            s += ' ' * margin
            s += ":"
            s += ' ' * margin
            s += desc.ljust(desc_col_width)
            s += ' ' * margin
            s += "|"
            rospy.loginfo(s)

        rospy.loginfo(end_line)

    def toggle_print_step_duration(self):
        self.print_step_duration = not self.print_step_duration

    def terminal_sim_control(self):
        """handles simulation controls via terminal"""

        seek_0 = lambda: self._seek_fraction(0)
        seek_1 = lambda: self._seek_fraction(0.1)
        seek_2 = lambda: self._seek_fraction(0.2)
        seek_3 = lambda: self._seek_fraction(0.3)
        seek_4 = lambda: self._seek_fraction(0.4)
        seek_5 = lambda: self._seek_fraction(0.5)
        seek_6 = lambda: self._seek_fraction(0.6)
        seek_7 = lambda: self._seek_fraction(0.7)
        seek_8 = lambda: self._seek_fraction(0.8)
        seek_9 = lambda: self._seek_fraction(0.9)

        key_map = defaultdict(
            lambda: (self.print_help), {
                " ": self.toggle_pause,
                "s": self.step_by_velodyne,
                "S": self.step_backwards_by_velodyne,
                "d": self.step_by_sick,
                "D": self.step_backwards_by_sick,
                "k": self.increase_playback_speed,
                "j": self.decrease_playback_speed,
                "0": seek_0,
                "1": seek_1,
                "2": seek_2,
                "3": seek_3,
                "4": seek_4,
                "5": seek_5,
                "6": seek_6,
                "7": seek_7,
                "8": seek_8,
                "9": seek_9,
                "b": self.toggle_print_step_duration,
            })

        while True:
            key_map[getch()]()

    # ------------------------------------------
    # SIMULATION CONTROL
    def unpause(self, _=None):
        # remember system time when simulation was resumed
        # --> sim time is computed based on that
        self.system_time_simulation_resumed = time.time()

        self.sim_paused = False

        rospy.loginfo(f"simulation RESUMED/STARTED")

        return []

    def pause(self, _=None):
        if self.sim_paused:
            return []

        self.sim_paused = True

        # remember what the simulation time was so that we know where to continue when unpausing
        self.sim_to_resume_at = self.sim_clock.clock.to_sec()

        rospy.loginfo("simulation PAUSED")

        return []

    def toggle_pause(self):
        if self.sim_paused:
            self.unpause()
        else:
            self.pause()

    def seek(self, request):
        self._seek(request.second)
        return []

    def _seek(self, second: float):
        """ effectively resets the timers and starts the simulation from
        timestamp `second` """
        # FIXME when the simulation is paused and we are seeking back in
        # time, the pointcloud is not being displayed and rviz, there are no
        # pointclouds received. when resuming everything is being shown though

        # remember system time when we resumed
        self.system_time_simulation_resumed = time.time()

        # set simulation to resume at the requested second
        self.sim_to_resume_at = second

        # update simulation and publish all things that need to be published at
        # this timestamp
        self._simulation_update()

    def step_by_velodyne(self, _=None):
        """ jumps to next available velodyne frame"""

        # FIXME: can't advance to next frame when sim_playback_speed is 0

        # first pause, then step
        self.pause()

        # looks for timestamp of the next available frame
        if self.last_published_frame + 1 <= self.timestamps_velodyne.shape[
                0] - 1:
            # move simulation to next frame
            self._seek(
                self.timestamps_velodyne.iloc[self.last_published_frame +
                                              1].to_sec())
        else:
            rospy.loginfo(
                "Can't advance to next velodyne frame. Simulation is at last frame."
            )

        return []

    def step_backwards_by_velodyne(self, _=None):
        """ jumps to previous velodyne frame"""

        # first pause if not paused
        self.pause()

        # check if we are already at frame 0
        if self.last_published_frame > 0:
            self._seek(
                self.timestamps_velodyne.iloc[self.last_published_frame -
                                              1].to_sec())
        else:
            rospy.loginfo(
                "Can't step to previous velodyne frame. Simulation already on first frame."
            )

        return []

    def step_by_sick(self, _=None):
        """ jumps to next available sick frame"""

        # FIXME: can't advance to next frame when sim_playback_speed is 0

        # first pause, then step
        self.pause()

        # looks for timestamp of the next available frame
        if self.last_published_sick_frame + 1 <= self.timestamps_sick_points.shape[
                0] - 1:
            # move simulation to next frame
            self._seek(
                self.timestamps_sick_points.iloc[self.last_published_sick_frame
                                                 + 1].to_sec())
        else:
            rospy.loginfo(
                "Can't advance to next sick frame. Simulation is at last frame."
            )

        return []

    def step_backwards_by_sick(self, _=None):
        """ jumps to previous frame"""

        # first pause, then step
        self.pause()

        # looks for timestamp of the next available frame
        if self.last_published_sick_frame > 0:
            # move simulation to next frame
            self._seek(
                self.timestamps_sick_points.iloc[self.last_published_sick_frame
                                                 - 1].to_sec())
        else:
            rospy.loginfo(
                "Can't step to previous sick frame. Simulation already on first frame."
            )

        return []

    def set_playback_speed(self, request):
        self._set_playback_speed(request.factor)
        return []

    def _set_playback_speed(self, factor):
        # set simulation timers as if we resumed now so that speed factor works

        # do not support negative playback speed
        if factor < 0:
            factor = 0

        # do not support playback speed > 10
        if factor > 10:
            factor = 10

        self._seek(self.sim_clock.clock.to_sec())
        self.sim_playback_speed = factor
        rospy.loginfo(f"playback speed factor set to {factor:.2f}")

    def set_looping(self, request):
        self.sim_looping = request.looping
        if request.looping:
            rospy.loginfo("LOOPING enabled")
        else:
            rospy.loginfo("LOOPING disabled")

        return []

    def increase_playback_speed(self, by=0.1):
        self._set_playback_speed(factor=round(self.sim_playback_speed +
                                              0.1, 2))

    def decrease_playback_speed(self, by=0.1):
        self._set_playback_speed(factor=round(self.sim_playback_speed -
                                              0.1, 2))

    def _seek_fraction(self, fraction):
        # 0 <= fraction <= 1
        assert fraction >= 0 and fraction <= 1, "invalid fraction in _seek_percent"
        self._seek((self.timestamps_velodyne.iloc[-1].secs -
                    self.timestamps_velodyne.iloc[0].secs) * fraction)

    # ------------------------------------------
    # HELPER
    def _get_maximum_timestamp(self):
        return self.timestamps_velodyne.iloc[-1]

    def _get_frame_to_be_published(self):
        # get last frame before current simulation time
        # NOTE this returns -1 if the simulation time is before the first frame
        return self.timestamps_velodyne.searchsorted(self.sim_clock.clock) - 1

    def _on_last_frame(self):
        """returns whether last published frame is the last frame of the simulation"""
        return self.last_published_frame == (
            self.timestamps_velodyne.shape[0] -
            1) and self.last_published_sick_frame == (
                self.timestamps_sick_points.shape[0] - 1)

    def _convert_frame_int_to_string(self, frame_int):
        if frame_int is None or frame_int == -1:
            return "----------"
        else:
            return '{:010d}'.format(frame_int)


# source: https://stackoverflow.com/a/7259460
def getch():
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

    try:
        while 1:
            try:
                c = sys.stdin.read(1)
                break
            except IOError:
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
    return c


if __name__ == "__main__":
    kdp = Kitti360DataPublisher()
    kdp.print_help()
    kdp.run()
