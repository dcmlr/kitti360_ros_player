import os
import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # must be set to true so that published /clock is used in rviz
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock')

    # simulation configuration
    rate_arg = DeclareLaunchArgument(
        name='rate',
        default_value='1',
        description='The playback speed as a factor')
    looping_arg = DeclareLaunchArgument(
        name='looping',
        default_value='True',
        description='Whether to loop back to start at the end')
    start_arg = DeclareLaunchArgument(
        name='start',
        default_value='0',
        description='Start N seconds into the simulation')
    end_arg = DeclareLaunchArgument(
        name='end',
        default_value='99999999',
        description='Stop N seconds into the simulation')
    sequence_arg = DeclareLaunchArgument(
        name='sequence',
        default_value='00',
        description='The KITTI-360 dataset sequence to play')
    directory_arg = DeclareLaunchArgument(
        name='directory',
        default_value='',
        description='The path to the KIITI-360 dataset directory The referenced directory should contain folders of the level `data_raw_3d`, `calibrations`, etc')

    # what data to show/load/publish or not - if disabled here, will also not be loaded into RAM
    pub_velodyne_arg = DeclareLaunchArgument(
        name='pub_velodyne',
        default_value='True',
        description='velodyne pointcloud')
    pub_velodyne_labeled_arg = DeclareLaunchArgument(
        name='pub_velodyne_labeled',
        default_value='True',
        description='velodyne pointcloud labeled')
    pub_sick_points_arg = DeclareLaunchArgument(
        name='pub_sick_points',
        default_value='True',
        description='sick points')
    pub_perspective_rectified_left_arg = DeclareLaunchArgument(
        name='pub_perspective_rectified_left',
        default_value='True',
        description='images of left perspective camera')
    pub_perspective_rectified_right_arg = DeclareLaunchArgument(
        name='pub_perspective_rectified_right',
        default_value='True',
        description='images of right perspective camera')
    pub_perspective_unrectified_left_arg = DeclareLaunchArgument(
        name='pub_perspective_unrectified_left',
        default_value='True',
        description='images of left perspective camera (unrectified)')
    pub_perspective_unrectified_right_arg = DeclareLaunchArgument(
        name='pub_perspective_unrectified_right',
        default_value='True',
        description='images of right perspective camera (unrectified)')
    pub_fisheye_left_arg = DeclareLaunchArgument(
        name='pub_fisheye_left',
        default_value='True',
        description='images of left fisheye camera')
    pub_fisheye_right_arg = DeclareLaunchArgument(
        name='pub_fisheye_right',
        default_value='True',
        description='images of right fisheye camera')
    pub_bounding_boxes_arg = DeclareLaunchArgument(
        name='pub_bounding_boxes',
        default_value='True',
        description='bounding boxes')
    pub_bounding_boxes_rviz_marker_arg = DeclareLaunchArgument(
        name='pub_bounding_boxes_rviz_marker',
        default_value='True',
        description='bounding boxes markers for rviz')
    pub_2d_semantics_left_arg = DeclareLaunchArgument(
        name='pub_2d_semantics_left',
        default_value='True',
        description='semantic ID of each pixel (left cam)')
    pub_2d_semantics_right_arg = DeclareLaunchArgument(
        name='pub_2d_semantics_right',
        default_value='True',
        description='semantic ID of each pixel (right cam)')
    pub_2d_semantics_rgb_left_arg = DeclareLaunchArgument(
        name='pub_2d_semantics_rgb_left',
        default_value='True',
        description='color-coded semantic label for each pixel (left cam)')
    pub_2d_semantics_rgb_right_arg = DeclareLaunchArgument(
        name='pub_2d_semantics_rgb_right',
        default_value='True',
        description='color-coded semantic label for each pixel (right cam)')
    pub_2d_instance_left_arg = DeclareLaunchArgument(
        name='pub_2d_instance_left',
        default_value='True',
        description='instance label of each pixel (left cam)')
    pub_2d_instance_right_arg = DeclareLaunchArgument(
        name='pub_2d_instance_right',
        default_value='True',
        description='instance label of each pixel (right cam)')
    pub_2d_confidence_left_arg = DeclareLaunchArgument(
        name='pub_2d_confidence_left',
        default_value='True',
        description='confidence map (left cam)')
    pub_2d_confidence_right_arg = DeclareLaunchArgument(
        name='pub_2d_confidence_right',
        default_value='True',
        description='confidence map (right cam)')
    pub_3d_semantics_static_arg = DeclareLaunchArgument(
        name='pub_3d_semantics_static',
        default_value='True',
        description='static 3d semantics')
    pub_3d_semantics_dynamic_arg = DeclareLaunchArgument(
        name='pub_3d_semantics_dynamic',
        default_value='True',
        description='dynamic 3d semantics')
    pub_camera_intrinsics_arg = DeclareLaunchArgument(
        name='pub_camera_intrinsics',
        default_value='True',
        description='camera intrinsics')

    params = [
        {'/use_sim_time': LaunchConfiguration('use_sim_time')},
        {'/kitti360_player/rate': LaunchConfiguration('rate')},
        {'/kitti360_player/looping': LaunchConfiguration('looping')},
        {'/kitti360_player/start': LaunchConfiguration('start')},
        {'/kitti360_player/end': LaunchConfiguration('end')},
        {'/kitti360_player/sequence': LaunchConfiguration('sequence')},
        {'/kitti360_player/directory': LaunchConfiguration('directory')},
        {'/kitti360_player/pub_velodyne': LaunchConfiguration('pub_velodyne')},
        {'/kitti360_player/pub_velodyne_labeled': LaunchConfiguration('pub_velodyne_labeled')},
        {'/kitti360_player/pub_sick_points': LaunchConfiguration('pub_sick_points')},
        {'/kitti360_player/pub_perspective_rectified_left': LaunchConfiguration('pub_perspective_rectified_left')},
        {'/kitti360_player/pub_perspective_rectified_right': LaunchConfiguration('pub_perspective_rectified_right')},
        {'/kitti360_player/pub_perspective_unrectified_left': LaunchConfiguration('pub_perspective_unrectified_left')},
        {'/kitti360_player/pub_perspective_unrectified_right': LaunchConfiguration('pub_perspective_unrectified_right')},
        {'/kitti360_player/pub_fisheye_left': LaunchConfiguration('pub_fisheye_left')},
        {'/kitti360_player/pub_fisheye_right': LaunchConfiguration('pub_fisheye_right')},
        {'/kitti360_player/pub_bounding_boxes': LaunchConfiguration('pub_bounding_boxes')},
        {'/kitti360_player/pub_bounding_boxes_rviz_marker': LaunchConfiguration('pub_bounding_boxes_rviz_marker')},
        {'/kitti360_player/pub_2d_semantics_left': LaunchConfiguration('pub_2d_semantics_left')},
        {'/kitti360_player/pub_2d_semantics_right': LaunchConfiguration('pub_2d_semantics_right')},
        {'/kitti360_player/pub_2d_semantics_rgb_left': LaunchConfiguration('pub_2d_semantics_rgb_left')},
        {'/kitti360_player/pub_2d_semantics_rgb_right': LaunchConfiguration('pub_2d_semantics_rgb_right')},
        {'/kitti360_player/pub_2d_instance_left': LaunchConfiguration('pub_2d_instance_left')},
        {'/kitti360_player/pub_2d_instance_right': LaunchConfiguration('pub_2d_instance_right')},
        {'/kitti360_player/pub_2d_confidence_left': LaunchConfiguration('pub_2d_confidence_left')},
        {'/kitti360_player/pub_2d_confidence_right': LaunchConfiguration('pub_2d_confidence_right')},
        {'/kitti360_player/pub_3d_semantics_static': LaunchConfiguration('pub_3d_semantics_static')},
        {'/kitti360_player/pub_3d_semantics_dynamic': LaunchConfiguration('pub_3d_semantics_dynamic')},
        {'/kitti360_player/pub_camera_intrinsics': LaunchConfiguration('pub_camera_intrinsics')}
    ]

    # created nodes
    package_name = 'kitti360_publisher'
    exec_name = 'publisher'
    kitti360_publisher = Node(
        package=package_name,
        executable=exec_name + '_node',
        name='custom_' + exec_name + '_node',
        output='screen',
        emulate_tty=True,
        parameters=params
    )

    gpsimu_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gpsimu_to_velodyne',
        arguments=['0.771049336280387', 
                   '0.29854143649499193', 
                   '-0.8362802189143268', 
                   '0.005805702483432155', 
                   '-0.010400477715954315', 
                   '3.1385789123483367',
                   'kitti360_gpsimu',
                   'kitti360_velodyne'])

    gps_imu_to_cam_00 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_imu_to_cam_00',
        arguments=['1.5752681039',
                   '0.0043914093',
                   '-0.65',
                   '1.5336079011307413',
                   '-0.009062306194569603',
                   '1.4722861590356016',
                   'kitti360_gpsimu',
                   'kitti360_cam_00'])

    gps_imu_to_cam_01 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_imu_to_cam_01',
        arguments=['1.59772414',
                   '0.59814949',
                   '-0.6488433108',
                   '1.5513935815559334',
                   '-0.012189496048847337',
                   '1.465665239810155',
                   'kitti360_gpsimu',
                   'kitti360_cam_01'])

    gps_imu_to_cam_02 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_imu_to_cam_02',
        arguments=['0.7264036936',
                   '-0.1499658517',
                   '-1.0686400091',
                   '-0.030797759202523006',
                   '0.003816106215825954',
                   '1.5606129768859012',
                   'kitti360_gpsimu',
                   'kitti360_cam_02'])

    gps_imu_to_cam_03 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_imu_to_cam_03',
        arguments=['0.7016842127',
                   '0.746365095',
                   '-1.0751978255',
                   '-3.1163865971589724',
                   '-0.0005898708748186543',
                   '1.5715637852794497',
                   'kitti360_gpsimu',
                   'kitti360_cam_03'])

    gps_imu_to_sick_points = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_imu_to_sick_points',
        arguments=['0.37316600787178855',
                   '0.3055298485560259',
                   '-0.7697157028904089',
                   '-0.007457061564034061',
                   '0.0021829398882018403',
                   '3.125319333492234',
                   'kitti360_gpsimu',
                   'kitti360_sick_points'])
    
    # add arguments and nodes to launch description
    return LaunchDescription([
        use_sim_time_arg,
        rate_arg,
        looping_arg,
        start_arg,
        end_arg, 
        sequence_arg,
        directory_arg,
        pub_3d_semantics_dynamic_arg,
        pub_3d_semantics_static_arg,
        pub_2d_confidence_left_arg,
        pub_2d_confidence_right_arg,
        pub_2d_instance_left_arg,
        pub_2d_instance_right_arg,
        pub_2d_semantics_left_arg,
        pub_2d_semantics_right_arg,
        pub_2d_semantics_rgb_left_arg,
        pub_2d_semantics_rgb_right_arg,
        pub_fisheye_left_arg,
        pub_fisheye_right_arg,
        pub_velodyne_arg,
        pub_velodyne_labeled_arg,
        pub_sick_points_arg,
        pub_perspective_rectified_left_arg,
        pub_perspective_rectified_right_arg,
        pub_perspective_unrectified_left_arg,
        pub_perspective_unrectified_right_arg,
        pub_bounding_boxes_arg,
        pub_bounding_boxes_rviz_marker_arg,
        pub_camera_intrinsics_arg,
        kitti360_publisher,
        gpsimu_to_velodyne,
        gps_imu_to_cam_00,
        gps_imu_to_cam_01,
        gps_imu_to_cam_02,
        gps_imu_to_cam_03,
        gps_imu_to_sick_points
    ])


if __name__ == '__main__':
    generate_launch_description()
