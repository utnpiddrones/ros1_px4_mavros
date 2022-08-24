#Al crear una nueva terminal, corra dentro del container
#   source /init.sh

#Ejecutar como usuario normal
docker exec -ti -u $(id -u) cont /bin/bash

#Ejecutar como SUDO
#docker exec -ti cont /bin/bash
