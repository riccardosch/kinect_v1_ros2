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

"""Kinect v1 camera in a multi-threaded component container (intra-process)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    qos_depth = LaunchConfiguration('qos_depth')
    log_buffer_address = LaunchConfiguration('log_buffer_address')
    intra_process = LaunchConfiguration('intra_process')
    bench = LaunchConfiguration('bench')

    intra_process_args = [
        {'use_intra_process_comms': ParameterValue(intra_process, value_type=bool)}]

    camera = ComposableNode(
        package='kinect_v1_driver',
        plugin='kinect_v1_driver::KinectCameraNode',
        name='kinect_camera',
        namespace=namespace,
        parameters=[{
            'qos_depth': ParameterValue(qos_depth, value_type=int),
            'log_buffer_address': ParameterValue(log_buffer_address, value_type=bool),
        }],
        extra_arguments=intra_process_args,
    )

    # Scenario A of the benchmark: consumer in the same container.
    image_stats = ComposableNode(
        package='kinect_v1_bench',
        plugin='kinect_v1_bench::ImageStatsNode',
        name='rgb_stats',
        namespace=namespace,
        remappings=[('image', 'rgb/image_raw')],
        parameters=[{
            'qos_depth': ParameterValue(qos_depth, value_type=int),
            'log_buffer_address': ParameterValue(log_buffer_address, value_type=bool),
        }],
        extra_arguments=intra_process_args,
        condition=IfCondition(bench),
    )

    container = ComposableNodeContainer(
        name='kinect_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera, image_stats],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='kinect'),
        DeclareLaunchArgument('qos_depth', default_value='1',
                              description='History depth of the sensor topics'),
        DeclareLaunchArgument('intra_process', default_value='true',
                              description='Enable intra-process (zero-copy) communication'),
        DeclareLaunchArgument('bench', default_value='false',
                              description='Load the image_stats consumer in the same container'),
        DeclareLaunchArgument('log_buffer_address', default_value='false',
                              description='Log every frame buffer address (zero-copy check)'),
        container,
    ])
