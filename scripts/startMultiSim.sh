#!/bin/bash

gnome-terminal \
	--tab -e "roslaunch mission8_sim multiDrone.launch" \
	--tab --working-directory ~/ardupilot/ArduCopter -e "sim_vehicle.py -j4 -f Gazebo --console --location=HOME -I0" \
	--tab --working-directory ~/ardupilot/ArduCopter -e "sim_vehicle.py -j4 -f Gazebo --console --location=HOME -I1" \
	