"""Real car: AMCL localization on the fixed track map + pure pursuit.

Bringup (VESC, lidar, joy, mux, odom TF) must already be running (started separately).

  ros2 launch ece484_racing car_pp.launch.py [map:=/abs/map.yaml] [waypoint_file:=/abs/wp.csv]
        [x:=0.0 y:=0.0 yaw:=0.21] [speed:=1.0 use_csv_speed:=true] [pp:=true]

x/y/yaw: AMCL initial pose in the map frame. Default 0,0,0 = where the mapping run
started (start_pp.sh passes the map's .start file, e.g. yaw 0.21 for track_fixed).
pp:=false starts localization only (check it in RViz before driving).
Pure pursuit only drives while RB (button 5) is held; it publishes speed 0 otherwise.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ece484_racing')
    amcl_params = os.path.join(share, 'config', 'amcl_car.yaml')
    pp_params = os.path.join(share, 'config', 'pp_car.yaml')
    f = lambda name: ParameterValue(LC(name), value_type=float)  # noqa: E731

    args = [
        DeclareLaunchArgument('map', default_value='/home/orin/sid/maps/track_fixed.yaml'),
        DeclareLaunchArgument('waypoint_file', default_value='/home/orin/sid/maps/car_track_fixed.csv'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('use_csv_speed', default_value='true'),
        DeclareLaunchArgument('lookahead_min', default_value='0.8'),
        DeclareLaunchArgument('pp', default_value='true', description='start pure pursuit'),
        DeclareLaunchArgument('drive_topic', default_value='/drive',
                              description='e.g. /pp_dryrun/drive for a no-drive test'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true only for bag replay tests on the laptop'),
    ]
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[amcl_params, sim, {'yaml_filename': LC('map')}])
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[amcl_params, sim, {'initial_pose.x': f('x'), 'initial_pose.y': f('y'),
                                  'initial_pose.yaw': f('yaw')}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen', parameters=[amcl_params, sim])
    pure_pursuit = Node(
        package='ece484_racing', executable='pure_pursuit', name='pure_pursuit', output='screen',
        condition=IfCondition(LC('pp')),
        parameters=[pp_params, sim, {
            'waypoint_file': LC('waypoint_file'),
            'drive_topic': LC('drive_topic'),
            'speed': f('speed'),
            'use_csv_speed': ParameterValue(LC('use_csv_speed'), value_type=bool),
            'lookahead_min': f('lookahead_min'),
        }])
    return LaunchDescription(args + [map_server, amcl, lifecycle, pure_pursuit])
