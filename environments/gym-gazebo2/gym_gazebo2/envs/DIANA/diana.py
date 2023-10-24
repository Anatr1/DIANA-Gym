from gym_gazebo2.utils import ut_generic, ut_launch
from ros2pkg.api import get_prefix_path
import numpy as np

# ROS 2
import rclpy
import os
import yaml
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSProfile
from gym_gazebo2.envs.DIANA.sensors import Sensors
import datetime
from pathlib import Path
import logging
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import json
#from gym import spaces
from gymnasium import spaces
import time
import math
import random
import subprocess
import gymnasium as gym
import psutil

import wandb

main_param = []
train_params = []

# Reward function hyperparameters
DISTANCE_MODIFIER = 600 #30
YAW_MODIFIER = 1.25 #1.2
GOAL_BONUS = 1000 #1000 
COLLISION_PENALTY = -400 
ZERO_ANGLE_BONUS = 0 # Should be higher than 0 for 3D environments 
ROLL_PITCH_MODIFIER = -661 
TIMEOUT_PENALTY = -0.5
SUBGOAL_DISTANCE_MODIFIER = 200


class DIANAEnv(gym.Env):
    def __init__(self, testing=False):
        """
        Initialize the DIANA environment
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        # self.realSpeed = True
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port
        
        # Launch robot in a new Process
        self.launch_subp = ut_launch.startLaunchServiceProcess(
            ut_launch.generateLaunchDescriptionDIANA(
                gzclient=self.gzclient,
                multiInstance=True,
                port=self.port,
            ))
        
        #print("Waiting for ROS to finish launching...")

        # Create the node after the new ROS_DOMAIN_ID is set in generateLaunchDescriptionDIANA
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)
        #print("\n\n\n\n\nROS node created!")

        train_params = self.parameters_declaration()
        self.param = self.get_param(testing)
       
        qos = QoSProfile(depth=10)
        self.node.sensors = Sensors(self.node)
        self.create_logdir(train_params['--policy'], main_param['sensor'], train_params['--logdir'])
        self.spin_sensors_callbacks()

        # Publishers
        self.node.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', qos) 

        self.reset_world_client     = self.node.create_client(Empty, 'reset_world')
        self.node.pause_physics_client   = self.node.create_client(Empty, 'pause_physics')
        self.node.unpause_physics_client = self.node.create_client(Empty, 'unpause_physics')

        self.episode_step       = 0
        self.episode            = 0
        self.collision_count    = 0
        self.t0                 = 0.0
        self.evaluate           = False
        self.index              = 0

        self.initial_pose, self.goals, self.poses = self.get_goals_and_poses()
        self.goal_pose = self.goals[0]
        self.robot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # [x, y, z, roll, pitch, yaw]

        self.cooldown = 0
        self.previous_subgoal = [0.0, 0.0]

        self.sent_subgoal = False
        self.last_sent_subgoal = [0.0, 0.0]
        self.second_last_sent_subgoal = [0.0, 0.0]

        self.node.get_logger().debug("DIANA Environment: Starting process")

        # Action space
        self.set_action_space()
  
        # Observation space
        self.set_observation_space()

        # start a new wandb run to track this script
        wandb.init(
            # set the wandb project where this run will be logged
            project="DIANA-Gym",
            
            # track hyperparameters and run metadata
            config={}
        )



    def step(self, action=None, reset_step=False, episode_step=0):
        #print("Stepping...")
        # At each step I have to reset the action space to take into
        # account the new position
        self.set_action_space()
        
        if action is not None:
            if self.cooldown > 0:
                #print ("Cooldown: ", self.cooldown)
                action = self.previous_subgoal
                self.sent_subgoal = False
            else:
                #print("Sending new subgoal, ", action)
                self.previous_subgoal = action

                self.second_last_sent_subgoal = self.last_sent_subgoal
                self.last_sent_subgoal = action
                self.sent_subgoal = True
                self.cooldown = 100 #500
                
            self.episode_step += 1

            self.node.get_logger().debug("sending action...")
            self.send_subgoal(action)

        self.node.get_logger().debug("getting sensor data...")
        self.spin_sensors_callbacks()
        depth_image, goal_info, self.robot_pose, collision = self.get_sensor_data()

        self.node.get_logger().debug("checking events...")
        done, event = self.check_events(goal_info, collision)

        terminated = False
        truncated = False
        if event == "collision" or event == "timeout":
            #print(f"Event: {event}. Truncated")
            truncated = True
        elif event == "goal":
            #print(f"Event: {event}. Terminated")
            terminated = True


        info = {}

        if not reset_step:
            self.node.get_logger().debug("getting reward...")
            reward = self.get_reward(goal_info, done, event)

            self.node.get_logger().debug("getting observation...")
            observation = self.get_observation(goal_info, depth_image)
        else:
            reward = None
            observation = None
            done = None

        self.previous_goal_info = goal_info

        #print("Episode: ", self.episode)
        #print("Episode step: ", self.episode_step+1, "/", self.node.timeout_steps)
        #print("Goal: ", self.goal_pose)
        #print("Goal distance: ", round(goal_info[0],3))
        #print("Goal angle: ", round(goal_info[1],3))
        #print(f"Current pose: X: {round(self.robot_pose[0], 3)}, Y: {round(self.robot_pose[1], 3)}, Z: {round(self.robot_pose[2], 3)}\
        #      \n              R: {round(self.robot_pose[3], 3)}, P: {round(self.robot_pose[4], 3)}, Y: {round(self.robot_pose[5], 3)}")
        #if reward is not None:
            #print("Reward: ", round(reward,2))
        #print("\n\n")

        # log metrics to wandb
        wandb.log({"reward": reward, "goal_distance": goal_info[0], "goal_angle": goal_info[1], "cooldown": self.cooldown})

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None):
        #print("Resetting environment...")
        time.sleep(2)
        self.episode += 1 # = n_episode
        #self.evaluate = evaluate
        #logging.info(f"Total_episodes: {'evaluate' if evaluate else n_episode}, Total_steps: {tot_steps}, episode_steps: {self.episode_step+1}\n")
        self.node.get_logger().info("Initializing new episode ...")
        logging.info("Initializing new episode ...")
        self.new_episode()
        self.node.get_logger().debug("Performing null step to reset variables")
        self.episode_step = 0

        info = {}
        _,_,_,_,_ = self.step(reset_step = True)
        observation,_,_,_,_ = self.step()

        return observation, info

    def render(self, mode='human'):
        pass

    def close(self):
        #print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()

    def create_logdir(self, policy, sensor, logdir):
        """
        """
        self.logdir = f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S.%f')}_{sensor}_{policy}/"
        Path(os.path.join(logdir, self.logdir)).mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=os.path.join(logdir, self.logdir, 'screen_logger.log'), 
            level=logging.INFO)
        
    def spin_sensors_callbacks(self):
        """
        """
        rclpy.spin_once(self.node)
        while None in self.node.sensors.sensor_msg.values():
            #print("Waiting for sensors to be ready...")
            #print(self.node.sensors.sensor_msg.keys())
            #print("\n\n\n\n\n")
            rclpy.spin_once(self.node)
            
        self.node.sensors.sensor_msg = dict.fromkeys(self.node.sensors.sensor_msg.keys(), None)

    def get_goals_and_poses(self):
        """
        """
        data = json.load(open(self.node.data_path,'r'))
        #print("Initial pose:" , data["initial_pose"])
        #print("Goals: ", data["goals"])
        #print("Poses: ", data["poses"])
        #time.sleep(10)
        return data["initial_pose"], data["goals"], data["poses"]
    
    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def send_subgoal(self, subgoal):
        # A subgoal is a 2D point in the x-y plane to be reached by the robot
        # in order to reach the final goal
        # Upon receiving a subgoal, the robot must first orient itself towards
        # the subgoal and then move towards it
        
        self.get_sensor_data()
        # Compute the angle between the robot's current position and the subgoal
        subgoal_dx = subgoal[0]-self.robot_pose[0]
        subgoal_dy = subgoal[1]-self.robot_pose[1]        

        path_theta = math.atan2(subgoal_dy, subgoal_dx)

        subgoal_angle = path_theta - self.robot_pose[5]

        if subgoal_angle > math.pi:
            subgoal_angle -= 2 * math.pi

        elif subgoal_angle < -math.pi:
            subgoal_angle += 2 * math.pi

        # Compute the distance between the robot's current position and the subgoal
        subgoal_distance = np.hypot(subgoal_dx, subgoal_dy)

        #print("Current subgoal: ", [round(subgoal[0],1), round(subgoal[1],1)])
        #print("Subgoal distance: ", round(subgoal_distance, 3))
        #print("Subgoal angle: ", round(subgoal_angle, 3))

        wandb.log({"subgoal_distance": subgoal_distance, "subgoal_angle": subgoal_angle})

        # Compute the twist to be sent to the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if math.fabs(subgoal_angle) > 0.05:
            twist.angular.z = 0.3 * np.sign(subgoal_angle)
        else:
            if subgoal_distance > 0.3:
                twist.linear.x = 1.5 #0.5

        self.send_action(twist)

        if subgoal_distance > 0.3:
            self.cooldown -= 1
        else:
            self.cooldown = 0

        time.sleep(0.1)

    def send_action(self, twist):
        #self.get_logger().debug("unpausing...")
        #self.unpause()

        #self.get_logger().debug("publishing twist...")
        self.node.cmd_vel_pub.publish(twist)
        self.compute_frequency()
        time.sleep(0.05)

        #self.get_logger().debug("pausing...")
        #self.pause()

    def compute_frequency(self,):
        t1=time.perf_counter()
        step_time = t1-self.t0
        self.t0 = t1
        twist_hz = 1./(step_time)
        self.node.get_logger().debug('Publishing Twist at '+str(twist_hz))

    def get_param(self, testing):
        if testing:
            configFilepath = os.path.join(
                get_package_share_directory("testing"), 'config',
                'main_params.yaml')
        else:
            configFilepath = os.path.join(
                get_package_share_directory("pic4rl"), 'config',
                'main_params.yaml')
                            
        # Load the topic parameters
        with open(configFilepath, 'r') as file:
            configParams = yaml.safe_load(file)['main_node']['ros__parameters']

        return configParams

    def parameters_declaration(self):
        global main_param, train_params

        goals_path       = os.path.join(
            get_package_share_directory('pic4rl'), 'goals_and_poses')
        main_param_path  = os.path.join(
            get_package_share_directory('pic4rl'), 'config', 'main_params.yaml')
        train_params_path= os.path.join(
            get_package_share_directory('pic4rl'), 'config', 'training_params.yaml')
        self.node.entity_path = os.path.join(
            get_package_share_directory("gazebo_sim"), 
            'models/goal_box/model.sdf'
            )
        
        #print("Goals path: ", goals_path)
        #print("Main params path: ", main_param_path)
        #print("Train params path: ", train_params_path)   
        #print("Entity path: ", self.node.entity_path)        

        with open(main_param_path, 'r') as main_param_file:
            main_param = yaml.safe_load(main_param_file)['main_node']['ros__parameters']
        with open(train_params_path, 'r') as train_param_file:
            train_params = yaml.safe_load(train_param_file)['training_params']


        #print("Main params: ", main_param)
        #print("Train params: ", train_params)

        self.node.declare_parameters(
            namespace   = '',
            parameters  = [
                ('data_path', main_param['data_path']),
                ('change_goal_and_pose', train_params['--change_goal_and_pose']),
                ('starting_episodes', train_params['--starting_episodes']),
                ('timeout_steps', train_params['--episode-max-steps']),
                ('robot_name', main_param['robot_name']),
                ('goal_tolerance', main_param['goal_tolerance']),
                ('lidar_dist', main_param['laser_param']['max_distance']),
                ('lidar_points', main_param['laser_param']['num_points']),
                ('policy', train_params['--policy']),
                ('policy_trainer', train_params['--policy_trainer']),
                ('max_lin_vel', main_param['max_lin_vel']),
                ('min_lin_vel', main_param['min_lin_vel']),
                ('max_ang_vel', main_param['max_ang_vel']),
                ('min_ang_vel', main_param['min_ang_vel']),
                ('gpu', train_params['--gpu']),
                ('batch_size', train_params['--batch-size']),
                ('n_warmup', train_params['--n-warmup'])
                ]
            )

        self.node.data_path      = self.node.get_parameter('data_path').get_parameter_value().string_value
        self.node.data_path      = os.path.join(goals_path, self.node.data_path)
        self.node.change_episode = self.node.get_parameter('change_goal_and_pose').get_parameter_value().integer_value
        self.node.starting_episodes = self.node.get_parameter('starting_episodes').get_parameter_value().integer_value
        self.node.timeout_steps  = self.node.get_parameter('timeout_steps').get_parameter_value().integer_value
        self.node.robot_name     = self.node.get_parameter('robot_name').get_parameter_value().string_value
        self.node.goal_tolerance = self.node.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.node.lidar_distance = self.node.get_parameter('lidar_dist').get_parameter_value().double_value
        self.node.lidar_points   = self.node.get_parameter('lidar_points').get_parameter_value().integer_value
        self.node.train_policy   = self.node.get_parameter('policy').get_parameter_value().string_value
        self.node.policy_trainer = self.node.get_parameter('policy_trainer').get_parameter_value().string_value
        self.node.min_ang_vel    = self.node.get_parameter('min_ang_vel').get_parameter_value().double_value
        self.node.min_lin_vel    = self.node.get_parameter('min_lin_vel').get_parameter_value().double_value
        self.node.max_ang_vel    = self.node.get_parameter('max_ang_vel').get_parameter_value().double_value
        self.node.max_lin_vel    = self.node.get_parameter('max_lin_vel').get_parameter_value().double_value
        self.node.gpu            = self.node.get_parameter('gpu').get_parameter_value().integer_value
        self.node.batch_size     = self.node.get_parameter('batch_size').get_parameter_value().integer_value
        self.node.n_warmup       = self.node.get_parameter('n_warmup').get_parameter_value().integer_value


        if self.node.train_policy == 'PPO':
            self.node.declare_parameters(namespace='',
            parameters=[
                ('horizon', train_params['--horizon']),
                ('normalize_adv', train_params['--normalize-adv']),
                ('enable_gae', train_params['--enable-gae'])
                ])

            self.node.horizon = self.node.get_parameter('horizon').get_parameter_value().integer_value
            self.node.normalize_adv = self.node.get_parameter('normalize_adv').get_parameter_value().bool_value
            self.node.enable_gae = self.node.get_parameter('enable_gae').get_parameter_value().bool_value

        else:
            self.node.declare_parameters(namespace='',
            parameters=[
                ('memory_capacity', train_params['--memory-capacity'])
                ])

            self.node.memory_capacity = self.node.get_parameter('memory_capacity').get_parameter_value().integer_value

        self.node.log_dict = {
            'policy': train_params['--policy'],
            'max_steps': train_params['--max-steps'],
            'max_episode_steps': train_params['--episode-max-steps'],
            'sensor': main_param['sensor'],
            'gpu': train_params['--gpu']
        }

        return train_params
    
    def new_episode(self):
        self.node.get_logger().debug("Resetting simulation ...")
        req = Empty.Request()

        while not self.reset_world_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('reset world service not available, waiting again...')
        self.reset_world_client.call_async(req)
        
        if self.episode % self.node.change_episode == 0. and not self.evaluate:
            self.index = int(np.random.uniform()*len(self.poses)) -1 

        self.node.get_logger().debug("Respawing robot ...")
        self.respawn_robot(self.index)
    
        self.node.get_logger().debug("Respawing goal ...")
        self.respawn_goal(self.index)

        self.node.get_logger().debug("Environment reset performed ...")

    def respawn_goal(self, index):
        #print("Respawning goal ...")
        #print("Ep:", self.episode)
        #print("Starting episodes:", self.node.starting_episodes)
        if self.episode <= self.node.starting_episodes: 
            self.get_random_goal()
        else:
            self.get_goal(index)

        self.node.get_logger().info(f"Ep {'evaluate' if self.evaluate else self.episode+1} goal pose [x, y]: {self.goal_pose}")
        logging.info(f"Ep {'evaluate' if self.evaluate else self.episode+1} goal pose [x, y]: {self.goal_pose}")

        position = "{x: "+str(self.goal_pose[0])+",y: "+str(self.goal_pose[1])+",z: "+str(0.01)+"}"
        pose = "'{state: {name: 'goal',pose: {position: "+position+"}}}'"
        subprocess.run(
            "ros2 service call /test/set_entity_state gazebo_msgs/srv/SetEntityState "+pose,
            shell=True,
            stdout=subprocess.DEVNULL
            )

        time.sleep(0.25)

    def get_goal(self, index):
        print("Next goal in the list:", self.goals[index])
        self.goal_pose = self.goals[index]

    def get_random_goal(self):
        if self.episode < 0 or self.episode % 25 == 0: # For the first 5 episodes, the goal is always the same (0.55, 0.55). Then, every 25 episodes, the goal is always the same
            x = 3.0 # 0.55
            y = 0.0 # 0.55
        else:
            x = random.randrange(-29, 29) / 10.0
            y = random.randrange(-29, 29) / 10.0

        x += self.initial_pose[0]
        y += self.initial_pose[1]

        #print("Generated random goal: ", x, y)
        self.goal_pose = [x, y]

    def get_sensor_data(self):
        sensor_data = {}
        sensor_data["odom"] = self.node.sensors.get_odom()
        _, lidar_collision = self.node.sensors.get_laser()        
        sensor_data["depth"], depth_camera_collision = self.node.sensors.get_depth()

        collision = lidar_collision #or depth_camera_collision
        
        if sensor_data["depth"] is None:
            sensor_data["depth"] = (np.zeros((112,112))).tolist()
        if sensor_data["odom"] is None:
            sensor_data["odom"] = [0.0,0.0,0.0,0.0,0.0,0.0]

        goal_info, robot_pose = self.process_odom(sensor_data["odom"])
        depth_image = sensor_data["depth"]

        return depth_image, goal_info, robot_pose, collision

    def process_odom(self, odom):
        goal_dx = self.goal_pose[0]-odom[0]
        goal_dy = self.goal_pose[1]-odom[1]

        goal_distance = np.hypot(goal_dx, goal_dy)

        path_theta = math.atan2(goal_dy, goal_dx)

        goal_angle = path_theta - odom[5]

        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        goal_info = [goal_distance, goal_angle]
        self.robot_pose = [odom[0], odom[1], odom[2], odom[3], odom[4], odom[5]]

        return goal_info, self.robot_pose

    def check_events(self, goal_info, collision):
        if collision:
            self.collision_count += 1
            if self.collision_count >= 3:
                self.collision_count = 0
                self.node.get_logger().info(f"Ep {'evaluate' if self.evaluate else self.episode+1}: Collision")
                logging.info(f"Ep {'evaluate' if self.evaluate else self.episode+1}: Collision")
                return True, "collision"
            else:
                return False, "None"

        if goal_info[0] < self.node.goal_tolerance:
            self.node.get_logger().info(f"Ep {'evaluate' if self.evaluate else self.episode+1}: Goal")
            logging.info(f"Ep {'evaluate' if self.evaluate else self.episode+1}: Goal")
            return True, "goal"

        if self.episode_step+1 >= self.node.timeout_steps:
            self.node.get_logger().info(f"Ep {'evaluate' if self.evaluate else self.episode+1}: Timeout")
            logging.info(f"Ep {'evaluate' if self.evaluate else self.episode+1}: Timeout")
            return True, "timeout"

        return False, "None"

    def get_reward(self, goal_info, done, event):
        # The reward is a function of the distance between the robot and the goal
        # and of the robot's orientation with respect to the goal
        reward = (self.previous_goal_info[0] - goal_info[0]) * DISTANCE_MODIFIER
        yaw_reward = (1-2*math.sqrt(math.fabs(goal_info[1]/math.pi))) * YAW_MODIFIER
        roll_reward  = ROLL_PITCH_MODIFIER * pow(self.robot_pose[3], 2) + ZERO_ANGLE_BONUS
        pitch_reward = ROLL_PITCH_MODIFIER * pow(self.robot_pose[4], 2) + ZERO_ANGLE_BONUS
        subgoal_reward = 0

        if self.sent_subgoal:
            # Compute euclidean distance between last sent subgoal and the global goal            
            subgoal_to_goal_distance = np.hypot(self.last_sent_subgoal[0]-self.goal_pose[0], self.last_sent_subgoal[1]-self.goal_pose[1])
            # Compute euclidean distance between second last sent subgoal and the global goal
            second_last_subgoal_to_goal_distance = np.hypot(self.second_last_sent_subgoal[0]-self.goal_pose[0], self.second_last_sent_subgoal[1]-self.goal_pose[1]) 
            subgoal_reward = (second_last_subgoal_to_goal_distance - subgoal_to_goal_distance) * SUBGOAL_DISTANCE_MODIFIER

        reward += yaw_reward + roll_reward + pitch_reward + subgoal_reward

        # Reward bonus and penalties for reaching the goal or colliding with an obstacle
        if event == "goal":
            reward += GOAL_BONUS
        elif event == "collision":
            reward += COLLISION_PENALTY

        # Reward penalty for taking too long to reach the goal
        if done:
            reward += (self.episode_step + 1) * TIMEOUT_PENALTY

        self.node.get_logger().debug(str(reward))

        return reward

    def get_observation(self, goal_info, depth_image):
        state_list = goal_info

        for point_line in depth_image:
            state_list.extend(point_line)

        state = np.array(state_list,dtype = np.float32)

        return state

    def respawn_robot(self, index):
        if self.episode <= self.node.starting_episodes:
            x, y, yaw = tuple(self.initial_pose)
        else:
            x, y , yaw = tuple(self.poses[index])

        qz = np.sin(yaw/2)
        qw = np.cos(yaw/2)

        self.node.get_logger().info(f"Ep {'evaluate' if self.evaluate else self.episode+1} robot pose [x,y,yaw]: {[x, y, yaw]}")
        logging.info(f"Ep {'evaluate' if self.evaluate else self.episode+1} robot pose [x,y,yaw]: {[x, y, yaw]}")

        position = "position: {x: "+str(x)+",y: "+str(y)+",z: "+str(0.07)+"}"
        orientation = "orientation: {z: "+str(qz)+",w: "+str(qw)+"}"
        pose = position+", "+orientation
        state = "'{state: {name: '"+self.node.robot_name+"',pose: {"+pose+"}}}'"
        subprocess.run(
            "ros2 service call /test/set_entity_state gazebo_msgs/srv/SetEntityState "+state,
            shell=True,
            stdout=subprocess.DEVNULL
            )
        time.sleep(0.25)

    def set_action_space(self):
        # For now we define the action space as a Box describing a square in the x-y plane 
        # centered on the robot's current position, with side length equal to 7 m.
        # The action space does not take into account the robot's orientation

        # WARNING: this implementation does not prevent the robot to exit map boundaries

        # Define the action space
        action =[
            [self.robot_pose[0] - 3.5, self.robot_pose[1] + 3.5], # x coordinate
            [self.robot_pose[1] - 3.5, self.robot_pose[1] + 3.5] # y coordinate
        ]
        low_action = []
        high_action = []
        for i in range(len(action)):
            low_action.append(action[i][0])
            high_action.append(action[i][1])
        low_action = np.array(low_action, dtype=np.float32)
        high_action = np.array(high_action, dtype=np.float32)

        self.action_space = spaces.Box(
            low=low_action,
            high=high_action,
            shape=(2,),
            dtype=np.float32
        )

    def set_observation_space(self):
        # The observation space is the current goal distance and goal angle
        # plus a 112x112 matrix containing the depth image.
        # Depth values range from 0 to 1
        state =[
            [0., 45.], # goal_distance
            [-math.pi, math.pi], # goal angle or yaw
        ]

        for i in range(self.param["depth_param"]["width"]*self.param["depth_param"]["height"]):
            state = state + [[0., 1]]

        if len(state)>0:
            low_state = []
            high_state = []
            for i in range(len(state)):
                low_state.append(state[i][0])
                high_state.append(state[i][1])
            self.low_state = np.array(low_state, dtype=np.float32)
            self.high_state = np.array(high_state, dtype=np.float32)
        
        self.observation_space = spaces.Box(
            low=self.low_state,
            high=self.high_state,
            dtype=np.float32
        )
