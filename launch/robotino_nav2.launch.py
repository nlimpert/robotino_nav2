# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Modified by Nicolas Limpert to be used for Festo Robotino 4

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robotino_nav2')
    bt_dir = get_package_share_directory('nav2_bt_navigator')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator']
#                       'map_server']

    # Get the robot-specific namespace from an environment variable
    # The actual namespace is unavailable at that point
    env_ns = os.environ.get('ROS_2_NAV_NS')
    env_id = os.environ.get('ROS_2_NAV_NS_ID')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/fawkes_scans/Laser_urg_filtered_360', '/' + env_ns + '/fawkes_scans/Laser_urg_filtered_360'),
                  ('/odom', '/' + env_ns + '/odom'),
                  ('/map', '/' + env_ns + '/map')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': env_ns + 'base_link',
        'id': env_id,
        'robotino_id': env_id,
        'robotino_frame': env_ns + 'base_link',
        'default_nav_to_pose_bt_xml': bt_dir + '/behavior_trees/navigate_to_pose_w_mapf_w_replanning_and_recovery.xml',
        'autostart': autostart}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

#    local_costmap_substitutions = param_substitutions + {'global_frame': env_ns + 'odom',
#                                  }

    # override the global frame for the local_costmap
    controller_param_substitutions = param_substitutions
    controller_param_substitutions.update({'global_frame': env_ns + 'odom'})

    controller_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=controller_param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),


        DeclareLaunchArgument(
            'namespace', default_value=env_ns,
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace', default_value='false',
            description='Whether to use the namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'robotino_nav2.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_params],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[controller_params],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])
