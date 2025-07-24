import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# パラメータを処理し、ノードを生成する関数
def launch_setup(context, *args, **kwargs):
    # --- ファイルパスの設定 ---
    pkg_share = get_package_share_directory('time_checker')
    param_file_path = os.path.join(pkg_share, 'config', 'turn_spot.yaml')

    # --- 引数の値を取得 ---
    # contextオブジェクトを使って、安全に引数の値を取得する
    location_str = context.launch_configurations['location']

    # --- YAMLを読み込み、パラメータを構築 ---
    with open(param_file_path, 'r') as f:
        all_params_yaml = yaml.safe_load(f)
    
    node_params_yaml = all_params_yaml['polygon_pass_opposite_line_time_measurer']['ros__parameters']

    final_params = {
        'vehicle_length': node_params_yaml.get('vehicle_length'),
        'vehicle_width': node_params_yaml.get('vehicle_width'),
        'base_link_to_center_offset': node_params_yaml.get('base_link_to_center_offset'),
    }

    try:
        keys = location_str.split('.')
        location_specific_params = node_params_yaml
        for key in keys:
            location_specific_params = location_specific_params[key]
        final_params.update(location_specific_params)
    except KeyError:
        raise RuntimeError(f"Location '{location_str}' not found in {param_file_path}")

    # --- ノードを定義して返す ---
    time_measurer_node = Node(
        package='time_checker',
        executable='polygon_pass_opposite_line_time_measurer',
        parameters=[final_params],
        output='screen'
    )
    
    return [time_measurer_node]

# メインのLaunchDescription生成関数
def generate_launch_description():
    # 起動時に場所名を指定する引数を宣言
    location_arg = DeclareLaunchArgument(
        'location',
        default_value='shiojiri.city_hall_entrance',
        description='Location key to use from the YAML file'
    )

    # OpaqueFunctionを使って、上記のlaunch_setup関数を呼び出す
    return LaunchDescription([
        location_arg,
        OpaqueFunction(function=launch_setup)
    ])
