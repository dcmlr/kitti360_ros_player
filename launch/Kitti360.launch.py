import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rate',
            default_value='1',
            description='The playback speed as a factor'
        ),
        launch.actions.DeclareLaunchArgument(
            name='looping',
            default_value='True',
            description='Whether to loop back to start at the end'
        ),
        launch.actions.DeclareLaunchArgument(
            name='start',
            default_value='0.0',
            description='Start N seconds into the simulation'
        ),
        launch.actions.DeclareLaunchArgument(
            name='end',
            default_value='99999999',
            description='Stop N seconds into the simulation'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sequence',
            default_value='00',
            description='The KITTI-360 dataset sequence to play'
        ),
        launch.actions.DeclareLaunchArgument(
            name='directory',
            default_value='',
            description='The path to the KIITI-360 dataset directory The referenced directory should contain folders of the level `data_raw_3d`, `calibrations`, etc'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_velodyne',
            default_value='True',
            description='velodyne pointcloud'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_velodyne_labeled',
            default_value='True',
            description='velodyne pointcloud labeled'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_sick_points',
            default_value='True',
            description='sick points'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_perspective_rectified_left',
            default_value='True',
            description='images of left perspective camera'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_perspective_rectified_right',
            default_value='True',
            description='images of right perspective camera'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_perspective_unrectified_left',
            default_value='True',
            description='images of left perspective camera (unrectified)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_perspective_unrectified_right',
            default_value='True',
            description='images of right perspective camera (unrectified)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_fisheye_left',
            default_value='True',
            description='images of left fisheye camera'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_fisheye_right',
            default_value='True',
            description='images of right fisheye camera'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_bounding_boxes',
            default_value='True',
            description='bounding boxes'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_bounding_boxes_rviz_marker',
            default_value='True',
            description='bounding boxes markers for rviz'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_semantics_left',
            default_value='True',
            description='semantic ID of each pixel (left cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_semantics_right',
            default_value='True',
            description='semantic ID of each pixel (right cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_semantics_rgb_left',
            default_value='True',
            description='color-coded semantic label for each pixel (left cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_semantics_rgb_right',
            default_value='True',
            description='color-coded semantic label for each pixel (right cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_instance_left',
            default_value='True',
            description='instance label of each pixel (left cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_instance_right',
            default_value='True',
            description='instance label of each pixel (right cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_confidence_left',
            default_value='True',
            description='confidence map (left cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_2d_confidence_right',
            default_value='True',
            description='confidence map (right cam)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_3d_semantics_static',
            default_value='True',
            description='static 3d semantics'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_3d_semantics_dynamic',
            default_value='True',
            description='dynamic 3d semantics'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pub_camera_intrinsics',
            default_value='True',
            description='camera intrinsics'
        ),
        launch_ros.actions.Node(
            package='kitti360_publisher',
            executable='core.py',
            name='kitti360_publisher',
            output='screen',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gpsimu_to_velodyne',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_imu_to_cam_00',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_imu_to_cam_01',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_imu_to_cam_02',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_imu_to_cam_03',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_imu_to_sick_points',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    '/kitti360_player/rate': launch.substitutions.LaunchConfiguration('rate')
                },
                {
                    '/kitti360_player/looping': launch.substitutions.LaunchConfiguration('looping')
                },
                {
                    '/kitti360_player/start': launch.substitutions.LaunchConfiguration('start')
                },
                {
                    '/kitti360_player/end': launch.substitutions.LaunchConfiguration('end')
                },
                {
                    '/kitti360_player/sequence': launch.substitutions.LaunchConfiguration('sequence')
                },
                {
                    '/kitti360_player/directory': launch.substitutions.LaunchConfiguration('directory')
                },
                {
                    '/kitti360_player/pub_velodyne': launch.substitutions.LaunchConfiguration('pub_velodyne')
                },
                {
                    '/kitti360_player/pub_velodyne_labeled': launch.substitutions.LaunchConfiguration('pub_velodyne_labeled')
                },
                {
                    '/kitti360_player/pub_sick_points': launch.substitutions.LaunchConfiguration('pub_sick_points')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_rectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_rectified_right')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_left': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_left')
                },
                {
                    '/kitti360_player/pub_perspective_unrectified_right': launch.substitutions.LaunchConfiguration('pub_perspective_unrectified_right')
                },
                {
                    '/kitti360_player/pub_fisheye_left': launch.substitutions.LaunchConfiguration('pub_fisheye_left')
                },
                {
                    '/kitti360_player/pub_fisheye_right': launch.substitutions.LaunchConfiguration('pub_fisheye_right')
                },
                {
                    '/kitti360_player/pub_bounding_boxes': launch.substitutions.LaunchConfiguration('pub_bounding_boxes')
                },
                {
                    '/kitti360_player/pub_bounding_boxes_rviz_marker': launch.substitutions.LaunchConfiguration('pub_bounding_boxes_rviz_marker')
                },
                {
                    '/kitti360_player/pub_2d_semantics_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_right')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_left': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_left')
                },
                {
                    '/kitti360_player/pub_2d_semantics_rgb_right': launch.substitutions.LaunchConfiguration('pub_2d_semantics_rgb_right')
                },
                {
                    '/kitti360_player/pub_2d_instance_left': launch.substitutions.LaunchConfiguration('pub_2d_instance_left')
                },
                {
                    '/kitti360_player/pub_2d_instance_right': launch.substitutions.LaunchConfiguration('pub_2d_instance_right')
                },
                {
                    '/kitti360_player/pub_2d_confidence_left': launch.substitutions.LaunchConfiguration('pub_2d_confidence_left')
                },
                {
                    '/kitti360_player/pub_2d_confidence_right': launch.substitutions.LaunchConfiguration('pub_2d_confidence_right')
                },
                {
                    '/kitti360_player/pub_3d_semantics_static': launch.substitutions.LaunchConfiguration('pub_3d_semantics_static')
                },
                {
                    '/kitti360_player/pub_3d_semantics_dynamic': launch.substitutions.LaunchConfiguration('pub_3d_semantics_dynamic')
                },
                {
                    '/kitti360_player/pub_camera_intrinsics': launch.substitutions.LaunchConfiguration('pub_camera_intrinsics')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
