from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='constant_velocity_prediction',
            executable='constant_velocity_prediction',
            name='cvm_prediction',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='people_publisher',
            executable='people_publisher',
            name='people_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='conflict_type_server',
            executable='conflict_type_server',
            name='conflict_type_server',
            parameters=[{'use_sim_time': use_sim_time}],
        )
        
        # Node(
        #     package='path_checker',
        #     executable='path_checker',
        #     name='path_checker')

        # Node(
        #     package='path_validity_check_server',
        #     executable='path_validity_check_server',
        #     name='path_validity_check_server',
        #     output='screen')
    ])