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

