#!/bin/bash
DIR="$(rospack find my_mavros)/scripts/config/cylinder.rviz"
rosrun rviz rviz -d $DIR