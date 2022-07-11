FROM px4io/px4-dev-ros-noetic:latest

##########################################################################
#   Instalación de paquetes de ROS (sacados del package.xml).
##########################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-geometry-msgs \
        ros-noetic-mavros-msgs \
        ros-noetic-geographic-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-mavros \
        ros-noetic-roscpp \
        ros-noetic-rospy \
        ros-noetic-tf2 \
        ros-noetic-tf2-ros \
        ros-noetic-tf2-tools \
        ros-noetic-gazebo-ros \
        ros-noetic-rtabmap-ros \
        ros-noetic-geodesy \
        ros-noetic-mavlink \
        ros-noetic-mavros-extras \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins \
        ros-noetic-rqt-robot-plugins \
        xmlstarlet \
    && rm -rf /var/lib/apt/lists/*

COPY ./my_mavros /home/user/catkin_my_mavros/src/my_mavros

##########################################################################
#   Instalación de GeographicLib
##########################################################################
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && . ./install_geographiclib_datasets.sh \
    && rm ./install_geographiclib_datasets.sh

##########################################################################
#   Instalación de QGroundControl
##########################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
		apt-utils \
		build-essential \
		ca-certificates \
		ccache \
		checkinstall \
		cmake \
		curl \
		espeak \
		fuse \
		g++ \
		gcc \
		git \
		gosu \
		kmod \
		libespeak-dev \
		libfontconfig1 \
		libfuse2 \
		libgstreamer-plugins-base1.0-dev \
		libgstreamer1.0-0 \
		libgstreamer1.0-dev \
		libsdl2-dev \
		libssl-dev \
		libudev-dev \
		locales \
		make \
		ninja-build \
		openssh-client \
		openssl \
		pkg-config \
		rsync \
		speech-dispatcher \
		wget \
		xvfb \
		zlib1g-dev \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        gstreamer1.0-gl \
	&& rm -rf /var/lib/apt/lists/*

COPY ./QGroundControl.AppImage /home/user/QGroundControl.AppImage

##########################################################################
#   Actualización de paquete (upgrade es necesario porque la version de
#	mavros se actualiza y los paquetes de ROS viejos no)
##########################################################################
RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

##########################################################################
#   Instalación de PX4
##########################################################################
COPY ./PX4-Autopilot /home/user/PX4-Autopilot

WORKDIR /home/user/PX4-Autopilot

RUN DONT_RUN=1 make px4_sitl_default gazebo

##########################################################################
#   Compilación de código de paquete propio
##########################################################################
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_my_mavros; rosdep update; rosdep install --from-paths src --ignore-src -r -y ./; catkin_make'

##########################################################################
#   Salida del container
##########################################################################
WORKDIR /home/user

COPY Docker/my_entrypoint.sh /
ENTRYPOINT ["/my_entrypoint.sh"]

CMD ["bash"]
