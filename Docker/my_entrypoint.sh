#!/bin/bash
#Estos comandos se ejecutarán cada vez que se ejecute el container

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/user/catkin_my_mavros/devel/setup.bash"
cd /home/user/catkin_my_mavros
catkin_make
cd /home/user

# Use the LOCAL_USER_ID if passed in at runtime
if [ -n "${LOCAL_USER_ID}" ]; then
	echo "Starting with UID : $LOCAL_USER_ID"
	# modify existing user's id
	usermod -u $LOCAL_USER_ID user
	# run as user
	exec gosu user "$@"
else
	exec "$@"
fi
