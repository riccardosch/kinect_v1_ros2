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

"""
Full Kinect v1 bringup in one multi-threaded, intra-process container.

Components: camera (RGB, depth, camera_info), aux (LED, IMU, microphones) and,
optionally, a depth_image_proc point cloud fed zero-copy from the camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

INTRA_PROCESS = [{'use_intra_process_comms': True}]


def as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes', 'on')


def point_cloud_node(ns, depth_mode, qos_depth):
    """depth_image_proc node matching the depth mode, or None."""
    # Same policy as the driver's sensor topics: latest cloud only, never block.
    points_qos = {'reliability': 'best_effort', 'depth': qos_depth}
    if depth_mode == 'mm':
        # XYZ in the depth camera frame. Subscribes image_rect + its sibling
        # camera_info with QoS matched to the publisher (best effort).
        # image_raw is used as image_rect: the default model has no distortion.
        topic = f'/{ns}/depth/points'
        return ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            name='point_cloud_xyz',
            namespace=ns,
            remappings=[('image_rect', 'depth/image_raw'),
                        ('camera_info', 'depth/camera_info'),
                        ('points', 'depth/points')],
            parameters=[{f'qos_overrides.{topic}.publisher.{k}': v
                         for k, v in points_qos.items()}],
            extra_arguments=INTRA_PROCESS,
        )
    if depth_mode == 'registered':
        # XYZRGB in the RGB camera frame. Its image subscriptions default to
        # RELIABLE (no match with our best-effort images): override them.
        topic = f'/{ns}/depth_registered/points'
        params = {f'qos_overrides.{topic}.publisher.{k}': v for k, v in points_qos.items()}
        for image in (f'/{ns}/depth_registered/image_raw', f'/{ns}/rgb/image_raw'):
            params[f'qos_overrides.{image}.subscription.reliability'] = 'best_effort'
        return ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='point_cloud_xyzrgb',
            namespace=ns,
            remappings=[('depth_registered/image_rect', 'depth_registered/image_raw'),
                        ('rgb/image_rect_color', 'rgb/image_raw'),
                        ('rgb/camera_info', 'rgb/camera_info'),
                        ('points', 'depth_registered/points')],
            parameters=[params],
            extra_arguments=INTRA_PROCESS,
        )
    return None


# Static TF of the sensor head, following the openni / rgbd_launch layout:
# kinect_link (REP 103: x forward, y left, z up) -> optical frames (z forward,
# x right, y down). Translations are nominal Kinect v1 values from rgbd_launch,
# not measured on this unit.
OPTICAL_ROTATION = {'x': -0.5, 'y': 0.5, 'z': -0.5, 'w': 0.5}  # rpy (-pi/2, 0, -pi/2)
SENSOR_FRAMES = [
    # (child frame, translation y [m], rotation)
    ('kinect_depth_optical_frame', -0.020, OPTICAL_ROTATION),
    ('kinect_rgb_optical_frame', -0.045, OPTICAL_ROTATION),
    ('kinect_imu_frame', 0.0, {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}),
]


def static_tf_nodes(ns):
    nodes = []
    for child, y, q in SENSOR_FRAMES:
        params = {'frame_id': 'kinect_link', 'child_frame_id': child,
                  'translation.x': 0.0, 'translation.y': y, 'translation.z': 0.0}
        params.update({f'rotation.{k}': v for k, v in q.items()})
        nodes.append(ComposableNode(
            package='tf2_ros',
            plugin='tf2_ros::StaticTransformBroadcasterNode',
            name=f'tf_{child}',
            namespace=ns,
            parameters=[params],
        ))
    return nodes


def launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    depth_mode = LaunchConfiguration('depth_mode').perform(context)
    qos_depth = int(LaunchConfiguration('qos_depth').perform(context))

    nodes = static_tf_nodes(ns) if as_bool(context, 'publish_tf') else []
    if as_bool(context, 'camera'):
        nodes.append(ComposableNode(
            package='kinect_v1_driver',
            plugin='kinect_v1_driver::KinectCameraNode',
            name='kinect_camera',
            namespace=ns,
            parameters=[{
                'enable_rgb': as_bool(context, 'enable_rgb'),
                'depth_mode': depth_mode,
                'qos_depth': qos_depth,
            }],
            extra_arguments=INTRA_PROCESS,
        ))
        if as_bool(context, 'point_cloud'):
            cloud = point_cloud_node(ns, depth_mode, qos_depth)
            if cloud is None:
                raise RuntimeError("point_cloud:=true needs depth_mode 'mm' or 'registered'")
            if depth_mode == 'registered' and not as_bool(context, 'enable_rgb'):
                raise RuntimeError('the registered (XYZRGB) point cloud needs enable_rgb:=true')
            nodes.append(cloud)
        if as_bool(context, 'remote_view'):
            # Compressed copies for a remote RViz: rgb/image_raw/compressed and
            # <depth ns>/image_raw/compressedDepth. Compression only runs while
            # someone subscribes to them.
            nodes.append(ComposableNode(
                package='kinect_v1_driver',
                plugin='kinect_v1_driver::CompressedRelay',
                name='compressed_relay',
                namespace=ns,
                parameters=[{
                    'rgb_ns': 'rgb' if as_bool(context, 'enable_rgb') else '',
                    'depth_ns': {'mm': 'depth', 'registered': 'depth_registered'}.get(
                        depth_mode, ''),
                    'jpeg_quality': int(LaunchConfiguration('jpeg_quality').perform(context)),
                    'depth_format': LaunchConfiguration('depth_format').perform(context),
                    'png_level': int(LaunchConfiguration('png_level').perform(context)),
                }],
                extra_arguments=INTRA_PROCESS,
            ))

    if as_bool(context, 'aux'):
        nodes.append(ComposableNode(
            package='kinect_v1_driver',
            plugin='kinect_v1_driver::KinectAuxNode',
            name='kinect_aux',
            namespace=ns,
            parameters=[{
                'imu_rate_hz': float(LaunchConfiguration('imu_rate_hz').perform(context)),
                'initial_led': LaunchConfiguration('initial_led').perform(context),
                'enable_audio': as_bool(context, 'enable_audio'),
                'publish_cancelled': as_bool(context, 'publish_cancelled'),
                'qos_depth': qos_depth,
            }],
            extra_arguments=INTRA_PROCESS,
        ))

    return [ComposableNodeContainer(
        name='kinect_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='kinect'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Load the camera component (RGB, depth, camera_info)'),
        DeclareLaunchArgument('aux', default_value='true',
                              description='Load the aux component (LED, IMU, microphones)'),
        DeclareLaunchArgument('enable_rgb', default_value='true'),
        DeclareLaunchArgument('depth_mode', default_value='mm',
                              description="'mm', 'registered' or 'off'"),
        DeclareLaunchArgument('point_cloud', default_value='false',
                              description='Load depth_image_proc in the container: depth/points '
                                          '(mm) or depth_registered/points (registered, XYZRGB)'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Static TF kinect_link -> optical / IMU frames '
                                          '(disable when a URDF provides them)'),
        DeclareLaunchArgument('remote_view', default_value='false',
                              description='Publish compressed images for a remote RViz '
                                          '(image_raw/compressed, image_raw/compressedDepth)'),
        DeclareLaunchArgument('jpeg_quality', default_value='80',
                              description='JPEG quality of rgb/image_raw/compressed [1-100]'),
        DeclareLaunchArgument('depth_format', default_value='rvl',
                              description="compressedDepth codec: 'rvl' (fast) or 'png'"),
        DeclareLaunchArgument('png_level', default_value='1',
                              description='PNG level when depth_format:=png [1-9]'),
        DeclareLaunchArgument('imu_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('initial_led', default_value='green'),
        DeclareLaunchArgument('enable_audio', default_value='true',
                              description='Publish the 4-microphone array (audio/data)'),
        DeclareLaunchArgument('publish_cancelled', default_value='false',
                              description="Also publish the Kinect's noise-cancelled channel"),
        DeclareLaunchArgument('qos_depth', default_value='1'),
        OpaqueFunction(function=launch_setup),
    ])
