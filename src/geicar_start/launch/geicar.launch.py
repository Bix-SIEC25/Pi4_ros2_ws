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

    online_logger = Node(
        package="online_logger",
        executable="online_logger",
        emulate_tty=True
    )

    image_sender = Node(
        package="online_logger",
        executable="image_sender",
        emulate_tty=True
    )

    socket_listener = Node(
        package="online_logger",
        executable="socket_listener",
        emulate_tty=True
    )

    watchdog = Node(
        package="watchdog",
        executable="watchdog",
        emulate_tty=True
    )

    safety_stop_node = Node(
        package="safety_stop",
        executable="safety_stop_node",
        parameters=[{
            "stop_dist_front_cm": 30,
            "stop_dist_rear_cm": 30,
            "slow_dist_front_cm": 55,
            "slow_dist_rear_cm": 55,
            "us_timeout_ms": 99999999999,
            "cmd_timeout_ms": 999,
            "log_actions": True
        }],
        emulate_tty=True
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

    audio_player_node = Node(
        package="audio_common",
        executable="audio_player_node",
        emulate_tty=True
    )

    tts_node = Node(
        package="audio_common",
        executable="tts_node",
        emulate_tty=True
    )

    music_node = Node(
        package="audio_common",
        executable="music_node",
        emulate_tty=True
    )

    # --- Logical order: RX -> control -> safety -> TX ---
    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(car_control_node)
    ld.add_action(online_logger)
    ld.add_action(safety_stop_node)
    ld.add_action(can_tx_node)
    # ld.add_action(imu_filter_madgwick_node)
    # ld.add_action(system_check_node)
    ld.add_action(audio_player_node)
    ld.add_action(tts_node)
    ld.add_action(music_node)
    #ld.add_action(watchdog)
    ld.add_action(socket_listener)
    ld.add_action(image_sender)

    return ld
