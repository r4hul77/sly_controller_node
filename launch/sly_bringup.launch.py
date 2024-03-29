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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, OpaqueFunction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
import launch
import launch_ros
from launch_ros.events.lifecycle import ChangeState

from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.events import matches_action
import copy
from lifecycle_msgs.msg import Transition
import xacro



def get_ros2_bag_launch():
    record_data_argument = DeclareLaunchArgument('record_data', default_value='true', description="set it to false to not to record data")
    home_loc = os.getenv('HOME')
    output_location = DeclareLaunchArgument('output_location', default_value=os.path.join(home_loc, 'ros_bag'), description="set the directory to record ros2_bag")

    def check_for_output_location(context, *args, **kwargs):
        output_location = LaunchConfiguration("output_location").perform(context=context)
        location = copy.copy(output_location)
        flag = True
        i = 1 
        while flag:
            flag = os.path.exists(location)
            if(flag):
                location = output_location + str(i)
                i += 1
        print(f"Writing to {location}")
        ros2_bag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record','-o', location, '-a'],
            condition=IfCondition(LaunchConfiguration('record_data'))
        )

        return [ros2_bag_process]

    ros2_bag = OpaqueFunction(function=check_for_output_location)


    return [record_data_argument, output_location, ros2_bag]

def get_gps_launch():
    params_dic = {'port':"/dev/ttyAMA1", 
                    'baud': 115200,
                    'frame_id':"gps",
                    'useRMC': False,
                    'time_ref_source':'gps'}
    driver_node =Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='log',
        parameters=[params_dic]
    )
    return [driver_node]

def get_robot_state_publisher_launch():

    pkg_sly_description = get_package_share_directory('sly_description')

    xacro_file = os.path.join(pkg_sly_description, "urdf", "sly_bot.xacro")

    robot_desc_xacro = xacro.process_file(xacro_file)
    
    robot_desc = robot_desc_xacro.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                'robot_description': robot_desc,
            }
        ]
    )

    return [robot_state_publisher_node]

def get_imu_launch():
    pkg_name = 'microstrain_inertial_driver'
    pkg_pth = get_package_share_directory(pkg_name)
    params_file = os.path.join(pkg_pth, "microstrain_inertial_driver_common", "config", "params.yml")

    params_dict = {
        'port': "/dev/ttyACM0",
    }

    microstrain_node = LifecycleNode(
        package = pkg_name,
        executable="microstrain_inertial_driver_node",
        name="imu_node",
        namespace="",
        parameters=[
                yaml.safe_load(open(params_file, 'r')),
                params_dict
        ]

    )

    config_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher= matches_action(microstrain_node),
            transition_id= Transition.TRANSITION_CONFIGURE,
        )
    )


    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(microstrain_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    imu_launch = [
        microstrain_node,
        config_event,
        activate_event
        ]

    return imu_launch




def generate_launch_description():
    pkg_motor_control = get_package_share_directory('motor_control')
    
    
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    # Gazebo launch
    motor_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_motor_control, 'launch', 'motor_control_launch.py'),
        )
    )

    teleop_launch = [
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ]

    ros_bag = launch.action

    # SLY Controller
    sly_controller_node = Node(
        package='sly_controller',
        executable='sly_controller_node',
        arguments=['--ros-args', '--log-level', 'warn'],
        output="log",

    )

    gps_launch = get_gps_launch()

    imu_launch = get_imu_launch()

    imu_lifecycle_manager = Node(
        package='sly_controller',
        executable='lifecycle_manager_node',
        output="screen",
    )

    robot_state_publisher = get_robot_state_publisher_launch()

    ros2_bag_launch = get_ros2_bag_launch()

    return LaunchDescription([
        motor_control,
        sly_controller_node,
        *teleop_launch,
        *gps_launch,
        *imu_launch,
        #imu_lifecycle_manager,
        *robot_state_publisher,
        *ros2_bag_launch,
    ])
