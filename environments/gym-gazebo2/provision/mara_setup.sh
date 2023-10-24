echo "Setting up mara environment"
echo "sourcing /opt/ros/humble/setup.zsh"
source /opt/ros/humble/setup.zsh
echo "sourcing ~/PATH/TO/MARA/WORKSPACE/install/setup.zsh"
source ~/PATH/TO/MARA/WORKSPACE/install/setup.zsh
echo "sourcing /usr/share/gazebo-11/setup.sh"
source /usr/share/gazebo-11/setup.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export RMW_IMPLEMENTATION=rmw_opensplice_cpp
export PYTHONPATH=$PYTHONPATH:~/PATH/TO/MARA/WORKSPACE/install/lib/python3/dist-packages
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/PATH/TO/MARA/WORKSPACE/install/share
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/PATH/TO/MARA/WORKSPACE/install/lib
