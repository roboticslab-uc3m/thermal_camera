from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

def generate_launch_description():

    data_name = DeclareLaunchArgument(
        "data_name", default_value=TextSubstitution(text="data0")
    )

    return LaunchDescription([
        Node(
            package='thermal_camera',
            namespace='camera_sub_pub',
            executable='camera_sub_pub',
            name='camera_sub_pub'
        ),
        Node(
            package='bag_recorder_nodes',
            namespace='simple_bag_recorder',
            executable='simple_bag_recorder',
            name='simple_bag_recorder',
            parameters=[{
                "data_name": LaunchConfiguration('data_name'),
            }]
        ),        
    ])
