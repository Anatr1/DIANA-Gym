import os
import sys
import time
from datetime import datetime

import gymnasium as gym
import gym_gazebo2
import tensorflow as tf
import multiprocessing

from importlib import import_module
from baselines import bench, logger
from baselines.ppo2 import ppo2
from stable_baselines3 import PPO, SAC
#from baselines.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.ppo.policies import MlpPolicy
from stable_baselines3.common.callbacks import EvalCallback

# Create save dir
save_dir = "./logs/"
os.makedirs(save_dir, exist_ok=True)

ncpu = multiprocessing.cpu_count()

if sys.platform == 'darwin':
    ncpu //= 2

print("Number of CPUs: ", ncpu)

config = tf.compat.v1.ConfigProto(allow_soft_placement=True,
                        intra_op_parallelism_threads=ncpu,
                        inter_op_parallelism_threads=ncpu,
                        log_device_placement=False)

config.gpu_options.allow_growth = True

tf.compat.v1.Session(config=config).__enter__()

def get_alg_module(alg, submodule=None):
    submodule = submodule or alg
    try:
        # first try to import the alg module from baselines
        alg_module = import_module('.'.join(['baselines', alg, submodule]))
    except ImportError:
        # then from rl_algs
        print("Import error, trying rl_algs")
        alg_module = import_module('.'.join(['rl_' + 'algs', alg, submodule]))

    print("Algorithm module: ", alg_module)
    
    return alg_module

def get_learn_function(alg, submodule=None):
    return PPO.learn

def get_learn_function_defaults(alg, env_type):
    try:
        alg_defaults = get_alg_module(alg, 'defaults')
        kwargs = getattr(alg_defaults, env_type)()
    except (ImportError, AttributeError):
        kwargs = {}

    print("Learn function defaults: ", kwargs)
    return kwargs

def make_env():
    env = gym.make(alg_kwargs['env_name'])
    env.set_episode_size(alg_kwargs['nsteps'])
    env = Monitor(env, filename=save_dir, allow_early_resets=True)

    print("Environment: ", env)
    return env

if __name__ == '__main__':
    multiprocessing.freeze_support()

    # Get dictionary from baselines/ppo2/defaults
    env_type = 'diana_mlp'
    alg_kwargs = get_learn_function_defaults('ppo2', env_type)
    print("alg_kwargs:", alg_kwargs)

    # Create needed folders
    timedate = datetime.now().strftime('%Y-%m-%d_%Hh%Mmin')    
    logdir = '/tmp/ros2learn/' + alg_kwargs['env_name'] + '/ppo2_mlp/' + timedate

    # Generate tensorboard file
    format_strs = os.getenv('MARA_LOG_FORMAT', 'stdout,log,csv,tensorboard').split(',')
    logger.configure(os.path.abspath(logdir), format_strs)


    with open(logger.get_dir() + "/parameters.txt", 'w') as out:
        out.write(
            'num_layers = ' + str(alg_kwargs['num_layers']) + '\n'
            + 'num_hidden = ' + str(alg_kwargs['num_hidden']) + '\n'
            + 'layer_norm = ' + str(alg_kwargs['layer_norm']) + '\n'
            + 'nsteps = ' + str(alg_kwargs['nsteps']) + '\n'
            + 'nminibatches = ' + str(alg_kwargs['nminibatches']) + '\n'
            + 'lam = ' + str(alg_kwargs['lam']) + '\n'
            + 'gamma = ' + str(alg_kwargs['gamma']) + '\n'
            + 'noptepochs = ' + str(alg_kwargs['noptepochs']) + '\n'
            + 'log_interval = ' + str(alg_kwargs['log_interval']) + '\n'
            + 'ent_coef = ' + str(alg_kwargs['ent_coef']) + '\n'
            + 'cliprange = ' + str(alg_kwargs['cliprange']) + '\n'
            + 'vf_coef = ' + str(alg_kwargs['vf_coef']) + '\n'
            + 'max_grad_norm = ' + str(alg_kwargs['max_grad_norm']) + '\n'
            + 'seed = ' + str(alg_kwargs['seed']) + '\n'
            + 'value_network = ' + alg_kwargs['value_network'] + '\n'
            + 'network = ' + alg_kwargs['network'] + '\n'
            + 'total_timesteps = ' + str(alg_kwargs['total_timesteps']) + '\n'
            + 'save_interval = ' + str(alg_kwargs['save_interval']) + '\n'
            + 'env_name = ' + alg_kwargs['env_name'] + '\n'
            + 'transfer_path = ' + str(alg_kwargs['transfer_path']) )

    # Create the environment
    env = SubprocVecEnv([make_env]) # FOR SINGLE PROCESS

    # FOR MULTIPLE PROCESSES
    #num_cpu = 2  # Number of processes to use
    # Create the vectorized environment
    #env = SubprocVecEnv([make_env for i in range(num_cpu)])

    model = PPO(MlpPolicy, env, verbose=0)

    #learn = get_learn_function('ppo2')
    transfer_path = alg_kwargs['transfer_path']

    # Remove unused parameters for training
    alg_kwargs.pop('env_name')
    alg_kwargs.pop('trained_path')
    alg_kwargs.pop('transfer_path')

    time.sleep(20)

    if transfer_path is not None:
        # Do transfer learning
        print("Loading model from: ", transfer_path)
        model = model.load(transfer_path)
    else:
        # Train from scratch
        print("Training from scratch")
        model.learn(total_timesteps=alg_kwargs['total_timesteps'])
        print("Learning finished")

    print("Closing environment")
    time.sleep(5)
    env.dummy().gg2().close()
    os.kill(os.getpid(), 9)

