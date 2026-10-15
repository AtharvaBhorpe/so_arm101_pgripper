from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_camera(context):
    """Resolve the device and configure capture before starting the driver."""
    device = str(
        Path(LaunchConfiguration('video_device').perform(context))
        .resolve(strict=True)
    )

    subprocess.run(
        [
            'v4l2-ctl',
            f'--device={device}',
            '--set-ctrl=exposure_dynamic_framerate=0',
        ],
        check=True,
        timeout=10,
    )

    config_dir = Path(
        get_package_share_directory('so_arm101_pgripper_cameras')
    ) / 'config'

    return [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='external_camera',
            parameters=[
                str(config_dir / 'external_camera.yaml'),
                {
                    'video_device': device,
                    'camera_info_url': (
                        config_dir / 'external_camera_calibration.yaml'
                    ).as_uri(),
                },
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    """Declare the external-camera device and launch capture."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-UGREEN_Camera_UGREEN_Camera_SN0001-video-index0'
            ),
        ),
        OpaqueFunction(function=launch_camera),
    ])
