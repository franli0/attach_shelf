from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.5',
        description='Distance to obstacle at which the robot will stop'
    )
    
    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='-90',
        description='Number of degrees for rotation after stopping'
    )
    
    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='true',
        description='Whether to perform the final approach after pre-approach'
    )
    
    # Define the nodes to launch
    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen'
    )
    
    pre_approach_v2_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2',
        name='pre_approach_v2_node',
        output='screen',
        parameters=[{
            'obstacle': LaunchConfiguration('obstacle'),
            'degrees': LaunchConfiguration('degrees'),
            'final_approach': LaunchConfiguration('final_approach')
        }]
    )
    
    # Launch RViz with appropriate configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [LaunchConfiguration('final_approach'), ' == "true" ',
                           ' && ', '$(find-pkg-share attach_shelf)/config/attach_to_shelf.rviz',
                           ' || ', '$(find-pkg-share attach_shelf)/config/pre_approach.rviz']]
    )
    
    # Return the launch description
    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        approach_service_server_node,
        pre_approach_v2_node,
        rviz_node
    ])