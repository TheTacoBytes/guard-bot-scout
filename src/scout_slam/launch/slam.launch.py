from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os, time

def generate_launch_description():
    # this is your package’s share directory
    pkg_share = get_package_share_directory('scout_slam')

    # Path to the SLAM Toolbox launch
    slam_toolbox_launch_dir = os.path.join(
        pkg_share, 'launch', 'online_async.launch.py')
    slam_params_file = os.path.join(
        pkg_share, 'params', 'mapper_params_online_async.yaml')

    # 1) EKF node: remap its /odometry/filtered output to /odom
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ os.path.join(pkg_share, 'config', 'ekf.yaml') ],
    #     remappings=[
    #         ('/odometry/filtered', '/odom'),
    #     ],
    # )

    # 2) Launch SLAM Toolbox, telling it to listen on /odom
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_dir),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        slam_toolbox_launch,
        # ekf_node,
        # rviz_node if you need it...
    ])


# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import os, time

# def generate_launch_description():
#     # this is your package’s share directory
#     pkg_share = get_package_share_directory('scout_slam')
#     # Path to the slam_toolbox launch file
#     slam_toolbox_launch_dir = os.path.join(
#         get_package_share_directory('scout_slam'), 'launch', 'online_async.launch.py')
#     # Path to the params file
#     slam_params_file = os.path.join(
#         pkg_share, 'params', 'mapper_params_online_async.yaml')
   
#     ekf_node = Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
#     )
#     # Include the SLAM Toolbox's online async launch file
#     slam_toolbox_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(slam_toolbox_launch_dir),
#         launch_arguments={
#             'use_sim_time': 'false',
#             'slam_params_file': slam_params_file,
#         }.items(),
#     )

#     # # Add the path to your existing broverette RViz config file from broverette_description
#     # rviz_config_file = os.path.join(
#     #     get_package_share_directory('scout_description'), 'rviz', 'none.rviz')

#     # # Node to launch RViz with the broverette.rviz configuration
#     # rviz_node = Node(
#     #     package='rviz2',
#     #     executable='rviz2',
#     #     name='rviz2',
#     #     output='screen',
#     #     arguments=['-d', rviz_config_file]
#     # )

#     return LaunchDescription([
#         slam_toolbox_launch,
#         ekf_node
#         # rviz_node
#     ])