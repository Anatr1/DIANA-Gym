import socket
import random
import os
import pathlib

from datetime import datetime
from billiard import Process

from ament_index_python.packages import get_package_prefix
from launch import LaunchService, LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python import get_package_share_directory
import yaml
from launch_ros.descriptions import ParameterValue
import time
import json

import gym_gazebo2
from gym_gazebo2.utils import ut_generic

MARA_PATH = "/PATH/TO/MARA/WORKSPACE" # TO BE MODIFIED

def startLaunchServiceProcess(launchDesc):
    """Starts a Launch Service process. To be called from subclasses.

    Args:
         launchDesc : LaunchDescription obj.
    """
    # Create the LauchService and feed the LaunchDescription obj. to it.
    launchService = LaunchService()
    launchService.include_launch_description(launchDesc)
    process = Process(target=launchService.run)
    #The daemon process is terminated automatically before the main program exits,
    # to avoid leaving orphaned processes running
    process.daemon = True
    process.start()

    return process

def isPortInUse(port):
    """Checks if the given port is being used.

    Args:
        port(int): Port number.

    Returns:
        bool: True if the port is being used, False otherwise.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket1:
        return socket1.connect_ex(('localhost', port)) == 0

def getExclusiveNetworkParameters():
    """Creates appropriate values for ROS_DOMAIN_ID and GAZEBO_MASTER_URI.

    Returns:
        Dictionary {ros_domain_id (string), ros_domain_id (string)}
    """

    randomPortROS = random.randint(0, 230)
    randomPortGazebo = random.randint(10000, 15000)
    while isPortInUse(randomPortROS):
        print("Randomly selected port is already in use, retrying.")
        randomPortROS = random.randint(0, 230)

    while isPortInUse(randomPortGazebo):
        print("Randomly selected port is already in use, retrying.")
        randomPortGazebo = random.randint(10000, 15000)

    # Save network segmentation related information in a temporary folder.
    tempPath = '/tmp/gym-gazebo-2/running/'
    pathlib.Path(tempPath).mkdir(parents=True, exist_ok=True)

    # Remove old tmp files.
    ut_generic.cleanOldFiles(tempPath, ".log", 2)

    filename = datetime.now().strftime('running_since_%H_%M__%d_%m_%Y.log')

    file = open(tempPath + '/' + filename, 'w+')
    file.write(filename + '\nROS_DOMAIN_ID=' + str(randomPortROS) \
        + '\nGAZEBO_MASTER_URI=http://localhost:' + str(randomPortGazebo))
    file.close()

    return {'ros_domain_id':str(randomPortROS),
            'gazebo_master_uri':"http://localhost:" + str(randomPortGazebo)}

def generateLaunchDescriptionDIANA(gzclient, multiInstance, port):
    """
        Returns ROS2 LaunchDescription object.
        Args:
            realSpeed: bool   True if RTF must be set to 1, False if RTF must be set to maximum.
    """

    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))

    print("\n\n\n\n\n\n Generating Launch Description DIANA")

    os.environ['AMENT_PREFIX_PATH'] = f'{MARA_PATH}/install:/opt/ros/humble:{root_dir}/resources/tb2_ugv:{root_dir}/resources/rosbot_ugv:{root_dir}/resources/pic4rl_testing:{root_dir}/resources/pic4rl:{root_dir}/resources/jackal_ugv:{root_dir}/resources/husky_ugv:{root_dir}/resources/gazebo_sim:{root_dir}/resources/cheddar_ugv:/opt/ros/humble'


    installDir = get_package_prefix('mara_gazebo_plugins')
    print("installDir: ", installDir)
    

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + installDir \
        + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = installDir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + installDir \
        + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = installDir + '/lib'

    if port != 11345: # Default gazebo port
        os.environ["ROS_DOMAIN_ID"] = str(port)
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(port)
        print("******* Manual network segmentation *******")
        print("ROS_DOMAIN_ID=" + os.environ['ROS_DOMAIN_ID'])
        print("GAZEBO_MASTER_URI=" + os.environ['GAZEBO_MASTER_URI'])
        print("")
    elif multiInstance:
        # Exclusive network segmentation, which allows to launch multiple instances of ROS2+Gazebo
        networkParams = getExclusiveNetworkParameters()
        os.environ["ROS_DOMAIN_ID"] = networkParams.get('ros_domain_id')
        os.environ["GAZEBO_MASTER_URI"] = networkParams.get('gazebo_master_uri')
        print("******* Exclusive network segmentation *******")
        print("ROS_DOMAIN_ID=" + networkParams.get('ros_domain_id'))
        print("GAZEBO_MASTER_URI=" + networkParams.get('gazebo_master_uri'))
        print("")

    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if key.isupper():
                envs[key] = os.environ[key]
    except BaseException as exception:
        print("Error with Envs: " + str(exception))
        return None
    
    print("Envs: ", envs)
    

    # Gazebo visual interface. GUI/no GUI options.
    if gzclient:
        gazeboCmd = "gazebo"
    else:
        gazeboCmd = "gzserver"

    print("gazeboCmd: ", gazeboCmd)
    
    ## Creation of ROS2 LaunchDescription obj.

    # Fetching Simulation Parameters from config file in gazebo_sim/config/sim_params.yaml
    # This file only contains the name of the package that contains the main_params.yaml
    simulation_configFilepath = os.path.join(
        get_package_share_directory("gazebo_sim"), 'config',
        'sim_params.yaml'
        )
    print("simulation_configFilepath: ", simulation_configFilepath)
    
    # Read the package name from the config file
    with open(simulation_configFilepath, 'r') as file:
        mode_package = yaml.safe_load(file)['sim_parameters']["package_name"]
    # mode_package:  pic4rl
    print("mode_package: ", mode_package)
    
    # Fetching Main Parameters from config file in pic4rl/config/main_params.yaml
    configFilepath = os.path.join(
        get_package_share_directory(mode_package), 'config',
        'main_params.yaml'
        )
    print("configFilepath: ", configFilepath)
    
    # Read the parameters from the config file
    with open(configFilepath, 'r') as file:
        configParams = yaml.safe_load(file)['main_node']['ros__parameters']
    print("configParams: ", configParams)
    
    # Fetching Goals and Poses from config file in pic4rl/goals_and_poses/new_indoor.json
    goals_path = os.path.join(
        get_package_share_directory(mode_package), 
        'goals_and_poses', 
        configParams['data_path']
        )
    print("goals_path: ", goals_path)
    
    goal_and_poses = json.load(open(goals_path,'r'))
    print("goal_and_poses: ", goal_and_poses)
    
    robot_pose, goal_pose = goal_and_poses["initial_pose"], goal_and_poses["goals"][0]
    print("robot_pose: ", robot_pose)
    print("goal_pose: ", goal_pose)
    
    x_rob = '-x '+str(robot_pose[0])
    y_rob = '-y '+str(robot_pose[1])
    z_rob = '-z '+str(0.3) 
    yaw_rob = '-Y ' +str(robot_pose[2])

    x_goal = '-x '+str(goal_pose[0])
    y_goal = '-y '+str(goal_pose[1])

    # Fetching Robot Package Name from config file in pic4rl/config/main_params.yaml
    robot_pkg = get_package_share_directory(configParams["robot_name"])
    print("robot_pkg: ", robot_pkg) 

    # Fetching Goal Entity from config file in gazebo_sim/models/goal_box/model.sdf
    goal_entity = os.path.join(get_package_share_directory("gazebo_sim"), 'models', 
                'goal_box', 'model.sdf')
    print("goal_entity: ", goal_entity)
    
    # For now we use a fixed world file
    worldPath = f"{root_dir}/environments/gym-gazebo2/gym_gazebo2/worlds/marsyard_ERC23.world"
    print("worldPath: ", worldPath)
    

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value = "true",
            description = 'Use simulation clock if true')
    print("use_sim_time_arg: ", use_sim_time_arg)
    
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
    		os.path.join(robot_pkg,'launch', 'description.launch.py')
            )
        )
    print("robot_description: ", robot_description)
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity',configParams["robot_name"], x_rob, y_rob, z_rob, yaw_rob, '-topic','/robot_description'],
    )
    print("spawn_robot: ", spawn_robot)

    spawn_goal = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'goal', '-file', goal_entity, x_goal, y_goal]
    )
    print("spawn_goal: ", spawn_goal)
    
    gazebo = ExecuteProcess(
        cmd=[gazeboCmd,'--verbose', worldPath, '-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],
        output='screen',
        env=envs
        )
    print("gazebo: ", gazebo)
    
    launchDesc = LaunchDescription([
        use_sim_time_arg,
        robot_description,
        spawn_robot,
        spawn_goal,
        TimerAction(period=0., actions=[gazebo]),
    ])
    print("launchDesc: ", launchDesc)
    
    print("LaunchDescription generated.")
    return launchDesc

def launchReal():
    os.environ["ROS_DOMAIN_ID"] = str(22)
    #os.environ["RMW_IMPLEMENTATION"] = "rmw_opensplice_cpp"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    installDir = get_package_prefix('mara_gazebo_plugins')
    launchDesc = LaunchDescription([
        Node(package='hros_cognition_mara_components',
             node_executable='hros_cognition_mara_components',
             arguments=["-motors", installDir \
             + "/share/hros_cognition_mara_components/motors.yaml", "real"], output='screen')
    ])
    return launchDesc
