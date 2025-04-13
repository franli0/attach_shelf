from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    
    # Define paths to RViz config files
    pkg_share = FindPackageShare('attach_shelf')
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'attach_to_shelf.rviz'])
    
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
        arguments=['-d', rviz_config_path]
    )
    
    # Return the launch description
    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        pre_approach_v2_node,
        final_approach_arg,
        approach_service_server_node,
        rviz_node
    ])