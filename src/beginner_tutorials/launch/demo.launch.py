from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    talker_freq = DeclareLaunchArgument(
        "freq", description="Frequency of publisher in talker", default_value=TextSubstitution(text="0.5")
    )
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
        parameters=[{
            "freq": LaunchConfiguration('freq')
        }]
    )
    listner_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='listener',
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

    return LaunchDescription([
        talker_freq,
        talker_node,
        listner_node
    ])
