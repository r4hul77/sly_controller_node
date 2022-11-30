# Copyright 2019 Louise Poubel
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

"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_motor_control = get_package_share_directory('motor_control')

    # Gazebo launch
    motor_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_motor_control, 'launch', 'motor_control_launch.py'),
        )
    )


    # SLY Controller
    sly_controller_node = Node(
        package='sly_controller',
        executable='sly_controller_node',
        arguments=[],
    )

    return LaunchDescription([
        motor_control,
        sly_controller_node,
    ])
