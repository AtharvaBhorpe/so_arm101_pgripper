from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_camera(context):
    device = str(Path(LaunchConfiguration('video_device').perform(context)).resolve(strict=True))

    subprocess.run(
        [
            'v4l2-ctl',
            f'--device={device}',
            '--set-ctrl=exposure_dynamic_framerate=0',
        ],
        check=True,
        timeout=10,
    )

    config = Path(
        get_package_share_directory('so_arm101_pgripper_cameras')
    ) / 'config' / 'gripper_camera.yaml'

    return [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='gripper_camera',
            output='screen',
            parameters=[str(config), {'video_device': device}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0'
            ),
        ),
        OpaqueFunction(function=launch_camera),
    ])
