from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ekf_params = os.path.join(
        get_package_share_directory('pioneer_robot'),
        'config',
        'ekf_params.yaml'
    )

    return LaunchDescription([

        # 1. Pioneer robot driver
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='aria_node',
            output='screen',
            arguments=['-rp', '/dev/ttyUSB0']
        ),

        # 2. Lakibeam1 lidar
        Node(
            package='lakibeam1',
            executable='lakibeam1_scan_node',
            name='lakibeam1_scan_node',
            output='screen',
            parameters=[{
                'frame_id':         'laser',
                'output_topic':     'scan',
                'inverted':         False,
                'hostip':           '0.0.0.0',
                'port':             '2368',
                'sensorip':         '192.168.198.2',
                'angle_offset':     0,
                'scanfreq':         '30',
                'filter':           '3',
                'laser_enable':     'true',
                'scan_range_start': '45',
                'scan_range_stop':  '315',
            }],
        ),

        # 2b. Static TF: laser → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),

        # 3. Joystick
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),

        # 4. Joystick controller (teleop override)
        Node(
            package='pioneer_robot',
            executable='joy_controller',
            name='joy_controller',
            output='screen'
        ),

        # 5. OAK-D driver
        Node(
            package='pioneer_robot',
            executable='oak_driver_node',
            name='oak_driver_node',
            output='screen'
        ),

        # 6. IMU — publishes /imu/data_raw
        Node(
            package='pioneer_robot',
            executable='phidgets_imu_node',
            name='phidgets_imu_node',
            output='screen'
        ),

        # 7. EKF — /odom + /imu/data_raw → /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),
    ])
