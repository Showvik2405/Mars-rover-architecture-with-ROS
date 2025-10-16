from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Launch argument to set initial mission mode
        DeclareLaunchArgument('mission_mode', default_value='science'),

        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Coordinator node
        Node(
            package='urc_missions',
            executable='coordinator',
            name='coordinator',
            parameters=[{'mission_mode': LaunchConfiguration('mission_mode')}]
        ),

        # Science mission node
        Node(
            package='urc_missions',
            executable='science',
            name='science'
        ),

        # Delivery server
        Node(
            package='urc_missions',
            executable='delivery_server',
            name='delivery_server'
        ),

        # Delivery client
        Node(
            package='urc_missions',
            executable='delivery_client',
            name='delivery_client'
        ),

        # Autonomous navigation
        Node(
            package='urc_missions',
            executable='autonav',
            name='autonav'
        ),

        # Vision node
        Node(
            package='urc_missions',
            executable='vision',
            name='vision'
        ),
    ])
