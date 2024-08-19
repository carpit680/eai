import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    eai_server = Node(
        package='eai',  # Replace with your service package name
        executable='main',  # Replace with your service node executable name
        output='screen'
    )

    gradio_client = Node(
        package='eai',  # Replace with your client package name
        executable='gradio_node',  # Replace with your client node executable name
        output='screen'
    )

    realsense_launch_file_dir = FindPackageShare('eai').find('eai')
    realsense_launch_file = os.path.join(realsense_launch_file_dir, 'rs_launch.py')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'base_frame_id': 'camera_link'
        }.items()
    )

    ld.add_action(eai_server)
    ld.add_action(gradio_client)
    ld.add_action(realsense_launch)
    return ld
