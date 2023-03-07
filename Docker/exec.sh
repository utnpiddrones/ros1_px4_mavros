#!/bin/bash

#Ejecutar como usuario normal
docker exec -ti -u "$(id -u)" cont /bin/bash -c "
    source /my_entrypoint.sh;
    rosrun my_mavros menu;"

#Ejecutar como SUDO
#docker exec -ti cont /bin/bash
