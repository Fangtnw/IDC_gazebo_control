import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_pkg_prefix = get_package_share_directory('gazebo')

    # Launch Gazebo
    gazebo_launch_file = os.path.join(gazebo_pkg_prefix, 'launch', 'gazebo.launch.py')
    gazebo_env = {
        'GAZEBO_MODEL_PATH': os.path.join(gazebo_pkg_prefix, 'models')
    }
    gazebo_node = Node(
        package='gazebo',
        executable='gazebo',
        output='screen',
        arguments=[gazebo_launch_file],
        env=gazebo_env
    )

    # Launch your keyboard plugin node
    keyboard_plugin_node = Node(
        package='gazebo_keyboard_plugin',
        executable='gazebo_keyboard_plugin',
        output='screen'
    )

    return LaunchDescription([
        gazebo_node,
        keyboard_plugin_node
    ])

if __name__ == '__main__':
    generate_launch_description()
