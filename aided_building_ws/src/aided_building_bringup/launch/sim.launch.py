import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def build_robot_state_publisher(context, xyz, rpy, use_sim_time):
    xyz_value = context.perform_substitution(xyz)
    rpy_value = context.perform_substitution(rpy)
    use_sim_time_value = context.perform_substitution(use_sim_time) == 'true'

    xacro_file = os.path.join(
        get_package_share_directory('aided_building_description'),
        'urdf',
        'franka_gazebo.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'xyz': xyz_value,
            'rpy': rpy_value,
        }).toxml()

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time_value,
        }],
    )]


def generate_launch_description():
    xyz = LaunchConfiguration('xyz')
    rpy = LaunchConfiguration('rpy')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_comm = LaunchConfiguration('start_comm')

    franka_share = get_package_share_directory('franka_description')
    description_share = get_package_share_directory('aided_building_description')
    resource_path = ':'.join(filter(None, [
        os.path.dirname(description_share),
        os.path.dirname(franka_share),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_franka',
        arguments=['-topic', '/robot_description', '-name', 'franka'],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=[
            'joint_state_broadcaster',
            'fr3_arm_controller',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(franka_share, 'rviz', 'visualize_franka.rviz'),
            '-f', 'world',
        ],
        condition=IfCondition(use_rviz),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
    )

    comm = Node(
        package='aided_building_comm',
        executable='comm_node',
        name='franka_comm',
        output='screen',
        condition=IfCondition(start_comm),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'xyz', default_value='0 0 0',
            description='Robot base position in the world'),
        DeclareLaunchArgument(
            'rpy', default_value='0 0 0',
            description='Robot base orientation in the world'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'start_comm', default_value='true',
            description='Start the simple ROS 2 command node'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        OpaqueFunction(
            function=build_robot_state_publisher,
            args=[xyz, rpy, use_sim_time]),
        spawn_robot,
        clock_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_controllers],
            )
        ),
        rviz,
        comm,
    ])
