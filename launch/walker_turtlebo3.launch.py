#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'),
                    'launch'),
                    '/turtlebot3_world.launch.py'
            ])
    ),
    Node(
        package='walker_turtlebot3',
        executable='walker_turtlebot3',
        name='walker_node'
    ),
    DeclareLaunchArgument(
      'rosbag_record',
      default_value = TextSubstitution(text = "True"),
      choices = ['True', 'False'],
      description = "Argument to enable/disable recording of all ros2 topics"
    ),
    ExecuteProcess(
      condition=IfCondition(LaunchConfiguration('rosbag_record')),
      cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
      shell=True
    )
  ])
