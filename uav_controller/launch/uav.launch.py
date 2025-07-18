from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('uav_controller')

    # 절대 경로로 YAML 파일 지정
    stable_params = os.path.join(pkg_share, 'config', 'stable_params.yaml')
    fast_params   = os.path.join(pkg_share, 'config', 'fast_params.yaml')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='stable',
        description='Choose "stable" or "fast" parameter set'
    )
    mode = LaunchConfiguration('mode')

    # mode 에 따라 파일 선택
    params_file = PythonExpression([
        "('", fast_params, "' if '", mode, "' == 'fast' else '", stable_params, "')"
    ])

    return LaunchDescription([
        mode_arg,
        Node(
            package='uav_controller',
            executable='uav_controller',
            name='uav_controller',
            output='screen',
            parameters=[ params_file ]
        )
    ])
