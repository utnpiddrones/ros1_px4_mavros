from ast import arguments
from distutils.cmd import Command
from multiprocessing.sharedctypes import Value
import os
from platform import node

from drivers.ament_index_python import get_package_share_directory

from drivers.launch import LaunchDescription
from drivers.launch.actions import DeclareLaunchArgument
from drivers.launch.actions import IncludeLaunchDescription
from drivers.launch.actions import GroupAction
from drivers.launch.launch_description_sources import PythonLaunchDescriptionSource
from drivers.launch.substitutions import LaunchConfiguration
from drivers.launch.substitutions import TextSubstitution
from drivers.launch_ros.actions import Node
from drivers.launch_ros.actions import PushRosNamespace

from drivers.launch.actions import set_environment_variable
from drivers.launch.substitutions import command
from drivers.launch.conditions import unless_condition
from drivers.launch.conditions import if_condition



def generate_launch_description():
    # args that can be set from the command line or a default will be used
    arg_ns = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="uav0")) #namespace del dron
    arg_id = DeclareLaunchArgument("ID", default_value=TextSubstitution(text="0")) #ID: debe ser un número entre 0 y 9

    arg_x = DeclareLaunchArgument("x", default_value=TextSubstitution(text="0")) #Posicion donde spawnea en la simulacion
    arg_y = DeclareLaunchArgument("y", default_value=TextSubstitution(text="0"))
    arg_z = DeclareLaunchArgument("z", default_value=TextSubstitution(text="0"))
    arg_R = DeclareLaunchArgument("R", default_value=TextSubstitution(text="0"))
    arg_P = DeclareLaunchArgument("P", default_value=TextSubstitution(text="0"))
    arg_yy = DeclareLaunchArgument("Y", default_value=TextSubstitution(text="0"))

    arg_vehicle = DeclareLaunchArgument("vehicle", default_value=TextSubstitution(text="iris")) #Modelo de PX4
    arg_sdf = DeclareLaunchArgument(
        "sdf", default_value=TextSubstitution(text="$(find my_mavros)/models/drone_utn/'drone_utn (kinect).sdf'")) #Path a sdf
    arg_est = DeclareLaunchArgument("est", default_value=TextSubstitution(text="ekf2")) #Tipo de estimador (filtro)

    arg_rtab = DeclareLaunchArgument("rtab", default_value=TextSubstitution(text="false"))#Inicia o no rtabmap
    arg_rviz = DeclareLaunchArgument("rviz", default_value=TextSubstitution(text="false"))
    ##No se si está bien
    arg_rvizcfg = DeclareLaunchArgument("rviz_cfg", default_value=TextSubstitution(text="$(find my_mavros)/scripts/rviz_config.rviz"))


    # include another launch file in the chatter_ns namespace
    push_ros_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('$(arg_ns)')),
            DeclareLaunchArgument("mavlink_udp_port", default_value=TextSubstitution(text="1456$(arg_ID)")),
            DeclareLaunchArgument("mavlink_tcp_port", default_value=TextSubstitution(text="456$(arg_ID)")),
            set_environment_variable("PX4_SIM_MODEL", value = LaunchConfiguration("$(arg_vehicle)")),
            set_environment_variable("PX4_ESTIMATOR", value = LaunchConfiguration("$(arg_est)")),
            Node(parameters=[{"model_description": command("xmlstarlet ed \
                -u '//plugin[@name=&quot;mavlink_interface&quot;]/mavlink_tcp_port' -v $(arg mavlink_tcp_port) \
                -u '//plugin[@name=&quot;mavlink_interface&quot;]/mavlink_udp_port' -v $(arg mavlink_udp_port) \
                -u '//plugin[@name=&quot;camera_plugin&quot;]/imageTopicName' -v /$(arg ns)/camera/color/image_raw \
                -u '//plugin[@name=&quot;camera_plugin&quot;]/cameraInfoTopicName' -v /$(arg ns)/camera/color/camera_info \
                -u '//plugin[@name=&quot;camera_plugin&quot;]/depthImageTopicName' -v /$(arg ns)/camera/depth/image_raw \
                -u '//plugin[@name=&quot;camera_plugin&quot;]/depthImageCameraInfoTopicName' -v /$(arg ns)/camera/depth/camera_info \
                -u '//plugin[@name=&quot;camera_plugin&quot;]/pointCloudTopicName' -v /$(arg ns)/camera/depth/points \
                -u '//plugin[@name=&quot;camera_plugin&quot;]/frameName' -v /$(arg ns)_camera_link \
                $(arg sdf)")}]),
            ##PX4 SITIL
            DeclareLaunchArgument("interactive", default_value=TextSubstitution(text="true")),
            DeclareLaunchArgument(unless_condition
                    (if_condition(LaunchConfiguration('$(arg interactive)'))),"px4_command_arg1",
                     value = LaunchConfiguration('')),
            DeclareLaunchArgument(if_condition(LaunchConfiguration('$(arg interactive)')), "px4_command_arg1",
                    value = LaunchConfiguration('-d')),
            Node(package='px4',
                name='sitl_$(arg ID)',
                arguments= '$(find px4)/build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i $(arg ID) \
                    -w sitl_$(arg vehicle)_$(arg ID) $(arg px4_command_arg1)',
                type="px4", #en ROS2 no existe type, cambiar
                output = 'screen' #en ROS2 no existe type, cambiar
            ),
            #Spawn vehicle
            Node(package='gazebo_ros',
                name='$(arg vehicle)_$(arg ID)_spawn',
                arguments= '-sdf -param model_description -model $(arg vehicle)_$(arg ID) -x $(arg x) -y $(arg y) -z $(arg z) \
                     -R $(arg R) -P $(arg P) -Y $(arg Y)',
                type="spawn_model", #en ROS2 no existe type, cambiar
                output = 'screen' #en ROS2 no existe type, cambiar
            ),
            #MAVROS
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('mavros'),'$/launch/px4.launch')),
                    DeclareLaunchArgument("fcu_url", value=LaunchConfiguration(text="udp://:1454$(arg ID)@localhost:1458$(arg ID)")),
                    DeclareLaunchArgument("gcs_url", value=LaunchConfiguration(text="")),
                    DeclareLaunchArgument("tgt_system", value=LaunchConfiguration(text="$(eval 1 + arg('ID'))")),
                    DeclareLaunchArgument("tgt_component", value=LaunchConfiguration(text="1"))
            ),
            #Arbol de transformadas
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('mavros'),'launch/topics/talker_listener.launch.py')),
                    DeclareLaunchArgument("ns", value=LaunchConfiguration(text="$(arg ns)"))
            ),
            #inicio RTab-Map
            IncludeLaunchDescription(
                if_condition(LaunchConfiguration('$(arg rtab)')),
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('mavros'),'/launch/rtab.launch')),
                    DeclareLaunchArgument("ns", value=LaunchConfiguration(text="$(arg ns)")),
                    DeclareLaunchArgument("rviz", value=LaunchConfiguration(text="$(arg rviz)")),
                    DeclareLaunchArgument("rviz_cfg", value=LaunchConfiguration(text="$(arg rviz_cfg)"))
            )
        ]
    )
    return LaunchDescription([
      arg_ns,arg_rtab, arg_rviz, arg_rvizcfg,
      arg_est, arg_id, arg_ns, arg_P, arg_R, 
      arg_sdf, arg_vehicle, arg_x, arg_y,
      arg_yy, arg_z, push_ros_namespace
    ])