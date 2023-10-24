import os
from ament_index_python.packages import get_package_share_directory
import launch
import yaml
import json
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import time

# Fetching Simulation Parameters from config file in gazebo_sim/config/sim_params.yaml
# This file only contains the name of the package that contains the main_params.yaml
simulation_configFilepath = os.path.join(
    get_package_share_directory("gazebo_sim"), 'config',
    'sim_params.yaml'
    )
# Read the package name from the config file
with open(simulation_configFilepath, 'r') as file:
    mode_package = yaml.safe_load(file)['sim_parameters']["package_name"]
# mode_package:  pic4rl

# Fetching Main Parameters from config file in pic4rl/config/main_params.yaml
configFilepath = os.path.join(
    get_package_share_directory(mode_package), 'config',
    'ARDITO_params.yaml'
    )
# Read the parameters from the config file
with open(configFilepath, 'r') as file:
    configParams = yaml.safe_load(file)['main_node']['ros__parameters']

# Fetching Goals and Poses from config file in pic4rl/goals_and_poses/new_indoor.json
goals_path = os.path.join(
    get_package_share_directory(mode_package), 
    'goals_and_poses', 
    configParams['data_path']
    )

goal_and_poses = json.load(open(goals_path,'r'))

robot_pose, goal_pose = goal_and_poses["initial_pose"], goal_and_poses["goals"][0]

x_rob = '-x '+str(robot_pose[0])
y_rob = '-y '+str(robot_pose[1])
z_rob = '-z '+str(0.3) # 0.3 is the height of the robot --> I DON'T LIKE THIS SOLUTION
yaw_rob = '-Y ' +str(robot_pose[2])

x_goal = '-x '+str(goal_pose[0])
y_goal = '-y '+str(goal_pose[1])
# z_goal = '-z 0.01' # NOT CLEAR IF THIS IS MEANT TO BE THE HEIGHT OF THE GOAL OR THE YAW, ANYWAY I DON'T USE IT

# Fetching World, model from config file in gazebo_sim/worlds/new_indoor.world
world_path = os.path.join(
    get_package_share_directory("gazebo_sim"), 
    'worlds', 
    configParams["world_name"]
    )

# Fetching Robot Package Name from config file in pic4rl/config/main_params.yaml
robot_pkg = get_package_share_directory(configParams["robot_name"])

# Fetching Goal Entity from config file in gazebo_sim/models/goal_box/model.sdf
goal_entity = os.path.join(get_package_share_directory("gazebo_sim"), 'models', 
            'goal_box', 'model.sdf')

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value = "true",
            description = 'Use simulation clock if true')

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
    		os.path.join(robot_pkg,'launch', 'description.launch.py')
            )
        )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity',configParams["robot_name"], x_rob, y_rob, z_rob, yaw_rob, '-topic','/robot_description'],
    )

    spawn_goal = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'goal', '-file', goal_entity, x_goal, y_goal]
    )
    
    gazebo = launch.actions.ExecuteProcess(
        # gzserver instead of gazebo to avoid the gui
        cmd=['gzserver','--verbose', world_path, '-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],
        output='screen'
        )
    
    return launch.LaunchDescription([
        use_sim_time_arg,
        robot_description,
        spawn_robot,
        spawn_goal,
        TimerAction(period=5., actions=[gazebo]),
    ])