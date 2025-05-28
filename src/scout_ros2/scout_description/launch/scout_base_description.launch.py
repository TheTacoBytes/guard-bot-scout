#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # allow switching to sim time if you ever need it
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # locate your URDF in the scout_description package
    urdf_file = os.path.join(
        get_package_share_directory('scout_description'),
        'urdf',
        'scout_mini',
        'scout_mini.urdf'
    )

    # read it straight into a string
    with open(urdf_file, 'r') as inf:
        robot_desc = inf.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time }]
        ),
        # print out which file we loaded
        LogInfo(msg=['Loading URDF from: ', urdf_file]),

        # publish TF from that URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                { 'use_sim_time': use_sim_time },
                { 'robot_description': robot_desc }
            ]
        ),
    ])




# #!/usr/bin/env python3

# import os

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, LogInfo
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     urdf_file = os.path.join(
#         get_package_share_directory('scout_description'),
#         'urdf',
#         'scout_mini',
#         'scout_mini.urdf'
#     )
#     with open(urdf_file, 'r') as inf:
#         robot_desc = inf.read()

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'
#         ),

#         LogInfo(msg=['Loading URDF from: ', urdf_file]),

#         # GUI to publish joint_states (all zeros by default)
#         Node(
#             package='joint_state_publisher_gui',
#             executable='joint_state_publisher_gui',
#             name='joint_state_publisher_gui',
#             output='screen',
#             parameters=[{ 'use_sim_time': use_sim_time }]
#         ),

#         # publishes all URDF → TF
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[
#                 { 'use_sim_time': use_sim_time },
#                 { 'robot_description': robot_desc }
#             ]
#         ),
#     ])







# import os
# import launch
# import launch_ros

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import FindExecutable, PathJoinSubstitution
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node


# def generate_launch_description():
#     model_name = 'scout_v2.xacro'
#     model_path = os.path.join(get_package_share_directory('scout_description'), "urdf", model_name)
#     print(model_path)
#     robot_description_content = Command([
#         PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
#         PathJoinSubstitution(
#             [FindPackageShare("scout_description"), "urdf", model_name]
#         ),
#     ])

#     return launch.LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='false',
#             description='Use simulation clock if true'),

#         launch.actions.LogInfo(msg='use_sim_time: '),
#         launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('use_sim_time')),
        
#         launch_ros.actions.Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{
#                 'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
#                 'robot_description':robot_description_content
#             }]),
#     ])
