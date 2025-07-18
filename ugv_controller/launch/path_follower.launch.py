from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_controller')
    
    default_mission_path = os.path.join(pkg_share, 'path', 'mission.csv')
    params_file_path = os.path.join(pkg_share, 'config', 'path_follower_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ) # 별도로 터미널에서 지정해주지 않으면 기본값인 true로 작동

    path_follower_node = Node(
        package='ugv_controller',
        executable='path_follower_node',
        name='path_follower',
        output='screen',
        parameters=[
            params_file_path,
            {'mission_file': default_mission_path},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        path_follower_node
    ])
