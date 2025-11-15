from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joystick",
        executable="joystick_ros2.py",
        emulate_tty=True
    )

    joystick_to_cmd_node = Node(
        package="joystick",
        executable="joystick_to_cmd",
        emulate_tty=True
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node",
        remappings=[('motors_order', 'motors_order_raw')],
        emulate_tty=True
    )

    safety_stop_node = Node(
        package="safety_stop",
        executable="safety_stop_node",
        parameters=[{
            "stop_dist_front_cm": 10,
            "stop_dist_rear_cm": 10,
            "us_timeout_ms": 200,
            "cmd_timeout_ms": 200,
            "log_actions": True
        }],
        emulate_tty=True
    )

    wheel_odom_node = Node(
        package="wheel_odom",
        executable="wheel_odom_node",
        parameters=[{
            "wheel_radius": 0.095,        # à ajuster
            "wheel_separation": 0.50,    # à ajuster
            "ticks_per_rev": 36,
            "frame_id": "odom",
            "child_frame_id": "base_link",
        }],
        emulate_tty=True
    )


    laser_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "-0.5", "0.0", "0.5",   # x, y, z (m)
            "0", "0", "0",          # roll, pitch, yaw (rad)
            "base_link",            # parent
            "laser"                 # enfant = frame du LIDAR
        ]
    )


    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True
    )

    config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
        emulate_tty=True
    )

    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True
    )

    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        parameters=[{
            "channel_type": "serial",
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": 256000,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True
        }],
        emulate_tty=True
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        emulate_tty=True
    )

    audio_capture_node = Node(
        package="audio_capture",
        executable="audio_capture_node",
        parameters=[{
            "device": "hw:1,0"
        }],
        emulate_tty=True
    )

    # --- Logical order: RX -> control -> safety -> TX ---
    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(car_control_node)
    ld.add_action(safety_stop_node)
    ld.add_action(wheel_odom_node)
    ld.add_action(laser_static_tf)
    ld.add_action(can_tx_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(system_check_node)
    ld.add_action(sllidar_node)
    ld.add_action(v4l2_camera_node)
    ld.add_action(audio_capture_node)

    return ld
