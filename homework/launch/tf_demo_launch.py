from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='ros2_tf_homework',
        executable='ros2_tf_homework',
        name='tf_system'
    )
    
    return LaunchDescription([node])