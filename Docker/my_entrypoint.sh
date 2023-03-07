#!/bin/bash
#Estos comandos se ejecutarán cada vez que se ejecute el container
source /init.sh
# Use the LOCAL_USER_ID if passed in at runtime
if [ "$(id -u)" -eq 0 ]; then
    cd /home/user/catkin_my_mavros
    catkin_make
    cd /home/user
	echo "Starting with UID : $LOCAL_USER_ID"
	# modify existing user's id
	usermod -u $LOCAL_USER_ID user
    wget https://github.com/TheAssassin/AppImageLauncher/releases/download/v2.2.0/appimagelauncher_2.2.0-travis995.0f91801.bionic_amd64.deb
	apt install ./appimagelauncher_2.2.0-travis995.0f91801.bionic_amd64.deb
fi

/bin/bash -c "$@"
