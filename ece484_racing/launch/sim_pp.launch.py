"""Pure pursuit in the sim.

  ros2 launch ece484_racing sim_pp.launch.py [speed:=1.5] [waypoint_file:=/abs/path.csv]
                                             [lookahead_min:=0.6] [static_map_odom:=true]
                                             [closed_loop:=false]

static_map_odom:=true publishes an identity map -> odom transform. Use it for
ground-truth testing (the sim's odom IS ground truth). Set it to false when
slam_toolbox or AMCL is running, because they publish map -> odom themselves.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ece484_racing')
    params = os.path.join(share, 'config', 'pp_sim.yaml')
    default_wp = os.path.join(share, 'waypoints', 'levine_lap.csv')

    args = [
        DeclareLaunchArgument('static_map_odom', default_value='true',
                              description='Publish identity map->odom (ground-truth testing)'),
        DeclareLaunchArgument('waypoint_file', default_value=default_wp),
        DeclareLaunchArgument('speed', default_value='1.5'),
        DeclareLaunchArgument('lookahead_min', default_value='0.6'),
        DeclareLaunchArgument('closed_loop', default_value='true',
                              description='false for an open path (stop at the last waypoint)'),
    ]

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=IfCondition(LaunchConfiguration('static_map_odom')),
    )
    pure_pursuit = Node(
        package='ece484_racing', executable='pure_pursuit', name='pure_pursuit', output='screen',
        parameters=[params, {
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'lookahead_min': ParameterValue(LaunchConfiguration('lookahead_min'), value_type=float),
            'closed_loop': ParameterValue(LaunchConfiguration('closed_loop'), value_type=bool),
        }],
    )
    return LaunchDescription(args + [static_tf, pure_pursuit])
