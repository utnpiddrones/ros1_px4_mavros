#Clono el repositorio de PX4 y lo pongo en la última version estable al momento de armar el container
cd ..
git clone https://github.com/PX4/PX4-Autopilot.git
#git clone https://github.com/OpenSLAM-org/openslam_g2o.git
cd PX4-Autopilot


cd ../Docker

#Buildeo la imagen de Docker
docker build \
    -t uav_slam_ros:1.0.0 \
    -f Dockerfile \
    ..

cd ..
rm -R PX4-Autopilot