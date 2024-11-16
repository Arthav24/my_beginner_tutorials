import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument to enable or disable bag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable or disable ROS 2 bag recording'
    )

    # Use a function to conditionally launch the bag recording
    def launch_rosbag_record(context, *args, **kwargs):
        if LaunchConfiguration('record_bag').perform(context) == 'true':
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a'],
                    output='screen'
                )
            ]
        return []

    return LaunchDescription([
        record_bag_arg,
        OpaqueFunction(function=launch_rosbag_record)
    ])
