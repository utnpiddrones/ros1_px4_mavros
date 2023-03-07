# Mavros_and_PX4

Simulación de cuatro cuadricópteros con el simulador Gazebo y el firmware PX4, recibiendo la información en ROS1 a través de Mavros.

## Ejecución

1. Ejecute el script para correr el container dentro de la carpeta Docker

    ```bash
    cd Docker
    ./run.sh
    ```

2. En terminales aparte, ejecute comandos dentro del container con:

    ```bash
    ./exec.sh
    ```

## Agregar grabación

Descargar la grabación desde esta carpeta [recording.bag](https://drive.google.com/drive/folders/1f3uM57X9pgJkICyFxJSS978hFiYU-TBA?usp=sharing) y ubicarla dentro de la carpeta `my_mavros/recordings/cylinder`. En esta carpeta también están los videos y las bases de datos de la simulación.

## Paper

* [SLAM con múltiples UAVs basado en cámaras RGB-D, odometrı́a inercial y GPS](https://www.researchgate.net/publication/359246340_SLAM_con_multiples_UAVs_basado_en_camaras_RGB-D_odometria_inercial_y_GPS).

* [Presentación usada en la JAR 2021](https://docs.google.com/presentation/d/1x5s9ssYrMX1Ae84zVlRfyac7R--dsn1KYMPbMbLxZDs/edit?usp=sharing).

Los datos guardados de la simulación ejecutada y de la formación de los mapas se encuentran en la siguiente [carpeta](https://drive.google.com/drive/folders/0B_pJsLitsr5LfkFnS3Jtc0I4S0lFeDg4V0g5cGZkSE9uWlo3N1paUWJ5Y2F4THJQREdZOFE?resourcekey=0-CHHLCWXkxk7UCQDFhNr7CQ&usp=sharing).
