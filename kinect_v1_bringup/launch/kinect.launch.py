# Copyright 2026 Riccardo Schiavo
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

"""Full Kinect v1 bringup: camera + aux (LED, IMU, microphones) in one intra-process container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    qos_depth = ParameterValue(LaunchConfiguration('qos_depth'), value_type=int)
    intra_process = [{'use_intra_process_comms': True}]

    camera = ComposableNode(
        package='kinect_v1_driver',
        plugin='kinect_v1_driver::KinectCameraNode',
        name='kinect_camera',
        namespace=namespace,
        parameters=[{
            'enable_rgb': ParameterValue(LaunchConfiguration('enable_rgb'), value_type=bool),
            # value_type=str is required: YAML would turn 'off' into False.
            'depth_mode': ParameterValue(LaunchConfiguration('depth_mode'), value_type=str),
            'qos_depth': qos_depth,
        }],
        extra_arguments=intra_process,
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    aux = ComposableNode(
        package='kinect_v1_driver',
        plugin='kinect_v1_driver::KinectAuxNode',
        name='kinect_aux',
        namespace=namespace,
        parameters=[{
            'imu_rate_hz': ParameterValue(LaunchConfiguration('imu_rate_hz'), value_type=float),
            'initial_led': ParameterValue(LaunchConfiguration('initial_led'), value_type=str),
            'enable_audio': ParameterValue(LaunchConfiguration('enable_audio'), value_type=bool),
            'publish_cancelled': ParameterValue(
                LaunchConfiguration('publish_cancelled'), value_type=bool),
            'qos_depth': qos_depth,
        }],
        extra_arguments=intra_process,
        condition=IfCondition(LaunchConfiguration('aux')),
    )

    container = ComposableNodeContainer(
        name='kinect_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera, aux],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='kinect'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Load the camera component (RGB + depth)'),
        DeclareLaunchArgument('aux', default_value='true',
                              description='Load the aux component (LED + IMU)'),
        DeclareLaunchArgument('enable_rgb', default_value='true'),
        DeclareLaunchArgument('depth_mode', default_value='mm',
                              description="'mm', 'registered' or 'off'"),
        DeclareLaunchArgument('imu_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('initial_led', default_value='green'),
        DeclareLaunchArgument('enable_audio', default_value='true',
                              description='Publish the 4-microphone array (audio/data)'),
        DeclareLaunchArgument('publish_cancelled', default_value='false',
                              description="Also publish the Kinect's noise-cancelled channel"),
        DeclareLaunchArgument('qos_depth', default_value='1'),
        container,
    ])
