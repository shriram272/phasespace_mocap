from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Dynamically find the path to the 'vis.rviz' file
    vis_rviz_path = FindPackageShare('phasespace_bringup').find('phasespace_bringup') + '/rviz/vis.rviz'

    return LaunchDescription([
        DeclareLaunchArgument(
            'server_ip',
            default_value='192.168.1.230',
            description='IP address of the motion capture server.'
        ),
        DeclareLaunchArgument(
            'display',
            default_value='true',
            description='Flag to display RViz.'
        ),
        Node(
            package='phasespace_bringup',
            executable='phasespace_bringup_node',
            name='phasespace_bringup',
            output='screen',
            parameters=[{'server_ip': LaunchConfiguration('server_ip')}]
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('display')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', vis_rviz_path],
            output='log'
        ),
        Node(
            package='phasespace_bringup',
            executable='phasespace_tf_visualization',
            name='phasespace_tf_visualization',
            output='screen'
        )
    ])
