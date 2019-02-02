# mission8_sim
Gazebo simulation of mission 8

## Setup 

run 
```
echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/mission8_sim/models' >> ~/.bashrc
```

## Running Sim 

```
cd ~/catkin_ws/src/mission8_sim/scripts
./startMultiSim.sh
```

## Controlling Actor

add the following line to the .bashrc

```
GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/tar/catkin_ws/build/mission8_sim/
```

## Running the actor sim

```
cd ~/catkin_ws/src/mission8_sim
roslaunch mission8_sim hello.launch
```

In another terminal run the key talker
```
cd ~/catkin_ws/src/mission8_sim
rosrun mission8_sim talker
```

## Files

hello.launch - launches the empty world with the actor

sim.launch - launches the arena with the actor (works really slowly)

sim.world - Gazebo world with the actor

hello.world - Gazebo world with just the actor

dalekControl.cpp - the plugin for the subscriber

keyTalker.cpp - the plugin for the keyboard controls

simple_world_plugin.cpp - the plugin for the actor simulation
