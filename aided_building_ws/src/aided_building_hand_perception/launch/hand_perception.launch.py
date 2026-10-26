import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from aided_building_hand_perception.geometry import (
    matrix_from_rpy, optical_frame_rotation, quaternion_from_matrix)


def static_tf(name, parent, child, translation, quaternion):
    x, y, z = (str(float(v)) for v in translation)
    qx, qy, qz, qw = (str(float(v)) for v in quaternion)
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
            '--frame-id', parent, '--child-frame-id', child,
        ],
        output='screen',
    )


def build_static_transforms(context, workbench_config, calibration_file):
    with open(context.perform_substitution(workbench_config)) as config_file:
        config = yaml.safe_load(config_file)
    calibration_path = os.path.expanduser(context.perform_substitution(calibration_file))

    base = config['robot_base_to_workbench']
    actions = [static_tf(
        'robot_base_to_workbench_tf', base['parent_frame'], base['child_frame'],
        base['translation'], quaternion_from_matrix(matrix_from_rpy(*base['rpy'])))]

    if os.path.exists(calibration_path):
        with open(calibration_path) as calibration_file_handle:
            camera = yaml.safe_load(calibration_file_handle)['workbench_to_camera']
        actions.append(LogInfo(msg=f'Workbench calibration: {calibration_path}'))
        actions.append(static_tf(
            'workbench_to_camera_tf', camera['parent_frame'], camera['child_frame'],
            camera['translation'], camera['rotation_xyzw']))
    else:
        fallback = config['fallback_camera']
        rotation = optical_frame_rotation(
            math.radians(fallback['yaw_deg']), math.radians(fallback['pitch_down_deg']))
        actions.append(LogInfo(
            msg=f'WARNING: {calibration_path} not found; using the rough fallback camera '
                f'pose from {context.perform_substitution(workbench_config)}. Positions '
                f'are only indicative until the ArUco calibration is done.'))
        actions.append(static_tf(
            'workbench_to_camera_tf', base['child_frame'], fallback['child_frame'],
            fallback['position'], quaternion_from_matrix(rotation)))
    return actions


def generate_launch_description():
    share = get_package_share_directory('aided_building_hand_perception')
    use_sim_time = LaunchConfiguration('use_sim_time')

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[
            LaunchConfiguration('camera_params'),
            {'video_device': LaunchConfiguration('video_device')},
        ],
        condition=IfCondition(LaunchConfiguration('start_camera')),
        output='screen',
    )

    detector = Node(
        package='aided_building_hand_perception',
        executable='hand_detector',
        name='hand_detector',
        parameters=[
            LaunchConfiguration('detector_params'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='Start the v4l2_camera driver'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='V4L2 device of the camera'),
        DeclareLaunchArgument(
            'camera_params', default_value=os.path.join(share, 'config', 'webcam.yaml'),
            description='v4l2_camera parameter file'),
        DeclareLaunchArgument(
            'detector_params', default_value=os.path.join(share, 'config', 'hand_detector.yaml'),
            description='hand_detector parameter file'),
        DeclareLaunchArgument(
            'workbench_config', default_value=os.path.join(share, 'config', 'workbench.yaml'),
            description='Robot base placement and fallback camera pose'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='~/.ros/aided_building/workbench_camera.yaml',
            description='Output of workbench_calibration (workbench -> camera)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set to true when running together with the Gazebo simulation'),

        camera,
        OpaqueFunction(
            function=build_static_transforms,
            args=[LaunchConfiguration('workbench_config'),
                  LaunchConfiguration('calibration_file')]),
        detector,
    ])
