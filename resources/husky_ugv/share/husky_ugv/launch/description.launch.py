import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
import launch_ros
import os
from ament_index_python import get_package_share_directory, get_package_prefix
import yaml
import xacro
from launch_ros.descriptions import ParameterValue

simulation_configFilepath = os.path.join(
    get_package_share_directory("gazebo_sim"), 'config',
    'sim_params.yaml'
    )
with open(simulation_configFilepath, 'r') as file:
    mode_package = yaml.safe_load(file)['sim_parameters']["package_name"]

def generate_launch_description():
    # Get the filepath to your config file
    configFilepath = os.path.join(
        get_package_share_directory(mode_package), 'config',
        'main_params.yaml'
    )                     
    # Load the parameters specific to your ComposableNode
    with open(configFilepath, 'r') as file:
        configParams = yaml.safe_load(file)['main_node']['ros__parameters']
    imu_arg = LaunchConfiguration('imu_enabled',
            default = configParams["imu_enabled"])
    camera_arg = LaunchConfiguration('camera_enabled',
            default = configParams["camera_enabled"])
    lidar_arg = LaunchConfiguration('lidar_enabled',
            default = configParams["lidar_enabled"])
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('husky_ugv')
    
    #default_model_path = os.path.join(pkg_share, 'urdf', 'husky.urdf.xacro')
    #robot_description_config = xacro.process_file(default_model_path)
    #robot_description = {"robot_description": robot_description_config.toxml()}
    robot_description_content = Command([
        PathJoinSubstitution([get_package_prefix('xacro'), 'bin', 'xacro']),
        ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'husky.urdf.xacro']),
        ' ',
        'imu_enabled:=', imu_arg,
        ' ',
        'camera_enabled:=', camera_arg,
        ' ',
        'laser_enabled:=', lidar_arg,
        ])

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": ParameterValue(robot_description_content, value_type=str)}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return launch.LaunchDescription([                          
        robot_state_publisher_node,
        joint_state_publisher_node
    ])