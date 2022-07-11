# Mavros_and_PX4

Implementación de la simulación de Gazebo con PX4, recibiendo la información a través de Mavros.

Se creo una imagen de Docker en el repositorio [ncotti/uav_slam_ros](https://hub.docker.com/r/ncotti/uav_slam_ros). La forma correcta de ejecutar esta simulación es a partir de este container, simplemente ejecutando el archivo `Docker/run.sh`.

# Paper

* [SLAM con múltiples UAVs basado en cámaras RGB-D, odometrı́a inercial y GPS](https://www.researchgate.net/publication/359246340_SLAM_con_multiples_UAVs_basado_en_camaras_RGB-D_odometria_inercial_y_GPS).

* [Presentación usada en la JAR 2021](https://docs.google.com/presentation/d/1x5s9ssYrMX1Ae84zVlRfyac7R--dsn1KYMPbMbLxZDs/edit?usp=sharing).

Los datos guardados de la simulación ejecutada y de la formación de los mapas se encuentran en la siguiente [carpeta](https://drive.google.com/drive/folders/0B_pJsLitsr5LfkFnS3Jtc0I4S0lFeDg4V0g5cGZkSE9uWlo3N1paUWJ5Y2F4THJQREdZOFE?resourcekey=0-CHHLCWXkxk7UCQDFhNr7CQ&usp=sharing).


# README original

## Instalación

1. Instale Docker https://docs.docker.com/get-docker/

2. Asegurese de que los permisos de Docker sean de super usuario https://docs.docker.com/engine/install/linux-postinstall/

3. Clone este repositorio.

```
$ git clone https://gitlab.frba.utn.edu.ar/utnbadrone/mavros_and_px4.git
```

4. Ejecute el script de instalación dentro de la carpeta Docker.
```
$ cd Docker && ./install.sh
```

## Ejecución

1. Ejecute el script para correr el container dentro de la carpeta Docker
```
$ ./run.sh
```

2. Notara que su terminal cambió. Ahora, todos los comandos que ejecute tendrán efecto exclusivamente dentro del contenedor. Como un atajo de interfaz de usuario, puede ejecutar:
```
$ rosrun my_mavros menu
```
Esta interfaz ejecuta los scripts que se encuentran dentro del container en la dirección "/home/user/catkin_my_mavros/src/my_mavros/scripts"

3. Si quiere ejecutar múltiples terminales para un mismo contenedor, utilize el script dentro de la carpeta "Docker" y, una vez dentro del contenedor debe ejecutar otro comando:
```
$ ./new_terminal.sh
$ source /init.sh   #Dentro del contenedor
```

4. Para terminar la sesión del contenedor, escriba en el terminal:
```
$ exit
```

## Archivos modificables por el usuario (fuera del container)
Los scripts de ejecución entro de la carpeta "my_mavros/scripts".

Los argumentos \<arg\> de los siguientes roslaunch. Estos tienen los paths donde guardar los distintos archivos, 
```
my_mavros/launch/main.launch
my_mavros/launch/rosbag.launch
```

## Workflow
Las modificaciones a los archivos fuente dentro de la carpeta "my_mavros" son actualizados en tiempo real dentro del Docker. La recompilación se realiza automáticamente al cerrar y volver a abrir el Docker.


## Datos
La versión de QGroundControl es la v4.1.4
El repositorio de PX4 está en la versión v1.12.3


# Documentación asociada y tutoriales

* [Presentación preliminar durante el desarrollo](https://docs.google.com/presentation/d/1lf9ulUvPaXQ3endR7WBigufnUrq0UwUEo1p6Jdc3DFk/edit?usp=sharing).
* [Notas personales y procedimiento](https://docs.google.com/document/d/1Vr04PHfXOuKhL9fF234AD3sMrTv1CPQf3MXd9ldpw_c/edit?usp=sharing).