# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    keyboard_node = Node(
        package='autonomy_utils',
        executable='keyboard_node',
        name='keyboard_node',
        output='screen',
        emulate_tty=True,  # Ensures it behaves like a terminal
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(keyboard_node)
    ld.add_action(joy_node)
    return ld