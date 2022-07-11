#En orden lo que hace cada línea

#Le da el nombre al container
#Conecta el display para ver las interfaces gráficas.
#Mismo número de usuario, necesario para las GUIs.            
#Conecta los drivers de video para las GUIs.
#Para ver las GUIs (como una FIFO)
#Volumen para el paquete catkin_my_mavros
#Volumen para el entrypoint (para poder modificarlo y no tener que buildear toda la imagen desde cero)
#Volumen para la entrada de nuevas terminales
#Nombre de la imagen sobre la que se crea el container

xhost +

docker run -i -t --rm --privileged \
       --name "cont" \
       -e DISPLAY=${DISPLAY} \
       -e LOCAL_USER_ID="$(id -u)" \
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
       -v "$(pwd)/../my_mavros:/home/user/catkin_my_mavros/src/my_mavros" \
       -v "$(pwd)/my_entrypoint.sh:/my_entrypoint.sh" \
       -v "$(pwd)/init.sh:/init.sh" \
       ncotti/uav_slam_ros:1.0.0 \
       /bin/bash


