import os

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
from drivers.launch.substitutions import command

from drivers.launch.action import SetLaunchConfiguration


def generate_launch_description():

#Coordenadas GPS del frame earth
     latitude_launch_arg = DeclareLaunchArgument("latitude", default_value=TextSubstitution(text="-34.660019"))
     longitude_launch_arg = DeclareLaunchArgument("longitude", default_value=TextSubstitution(text="-58.468302"))
     amsl_launch_arg = DeclareLaunchArgument("amsl", default_value=TextSubstitution(text="14"))

    #vehiculo y mundo a ejecutar
     world_launch_arg = DeclareLaunchArgument("world", default_value=TextSubstitution(text=os.path.join(get_package_share_directory('my_ros'),'worlds/cylinder.world')))
     est_launch_arg = DeclareLaunchArgument("est", default_value=TextSubstitution(text="ekf2"))
     vehicle_launch_arg = DeclareLaunchArgument("vehicle", default_value=TextSubstitution(text="iris"))
     sdf_launch_arg = DeclareLaunchArgument("sdf", default_value=TextSubstitution(text=os.path.join(get_package_share_directory('my_ros'),'models/drone_utn/drone_utn (kinect).sdf')))

     #configuracion del gazebo     
     gui_launch_arg = DeclareLaunchArgument("gui", default_value=TextSubstitution(text="true"))
     debug_launch_arg = DeclareLaunchArgument("debug", default_value=TextSubstitution(text="false"))
     verbose_launch_arg = DeclareLaunchArgument("verbose", default_value=TextSubstitution(text="false"))
     paused_launch_arg = DeclareLaunchArgument("paused", default_value=TextSubstitution(text="false"))


    # Configuraciones de RVIZ (Note: para la visualización, es necesario que rtab del uav0 esté en true)-->         
     rviz_launch_arg = DeclareLaunchArgument("rviz", default_value=TextSubstitution(text="true"))
     rviz_cfg_launch_arg = DeclareLaunchArgument("rviz_cfg", default_value=TextSubstitution(text="$(find my_mavros)/scripts/config/cylinder.rviz"))

    #<!--Inicializa los drones-->
     uav0_launch_include = IncludeLaunchDescription(
        
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_ros'),'launch/spawn_dron.launch')),

        DeclareLaunchArgument("ns",default_value=TextSubstitution(text="uav0")),#no me cierra el declare acá
        DeclareLaunchArgument("ID",default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("vehicle",default_value=TextSubstitution(text="$(arg vehicle)")),
        DeclareLaunchArgument("sdf",default_value=TextSubstitution(text="$(arg sdf)")),
        DeclareLaunchArgument("est", default_value=TextSubstitution(text="$(arg est)")),
        DeclareLaunchArgument("rtB",default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("x", dafault_value=TextSubstitution(text="1")),
        DeclareLaunchArgument("y", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("z", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("R", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("P", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("Y", dafault_value=TextSubstitution(text="0")),

        DeclareLaunchArgument("rviz",default_value=TextSubstitution(text="$(arg rviz)")),
        DeclareLaunchArgument("rviz_cfg",default_value=TextSubstitution(text="$(arg rviz_cfg)"))
     )

     uav1_launch_include = IncludeLaunchDescription(
       
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_ros'),'launch/spawn_dron.launch')),

        DeclareLaunchArgument("ns",default_value=TextSubstitution(text="uav1")),#no me cierra el declare acá
        DeclareLaunchArgument("ID",default_value=TextSubstitution(text="1")),
        DeclareLaunchArgument("vehicle",default_value=TextSubstitution(text="$(arg vehicle)")),
        DeclareLaunchArgument("sdf",default_value=TextSubstitution(text="$(arg sdf)")),
        DeclareLaunchArgument("est", default_value=TextSubstitution(text="$(arg est)")),
        DeclareLaunchArgument("rtB",default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("x", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("y", dafault_value=TextSubstitution(text="1")),
        DeclareLaunchArgument("z", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("R", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("P", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("Y", dafault_value=TextSubstitution(text="1.5710")),

   
     )

     uav2_launch_include = IncludeLaunchDescription(
       
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_ros'),'launch/spawn_dron.launch')),

        DeclareLaunchArgument("ns",default_value=TextSubstitution(text="uav2")),#no me cierra el declare acá
        DeclareLaunchArgument("ID",default_value=TextSubstitution(text="2")),
        DeclareLaunchArgument("vehicle",default_value=TextSubstitution(text="$(arg vehicle)")),
        DeclareLaunchArgument("sdf",default_value=TextSubstitution(text="$(arg sdf)")),
        DeclareLaunchArgument("est", default_value=TextSubstitution(text="$(arg est)")),
        DeclareLaunchArgument("rtB",default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("x", dafault_value=TextSubstitution(text="-1")),
        DeclareLaunchArgument("y", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("z", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("R", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("P", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("Y", dafault_value=TextSubstitution(text="3.1416")),

   
     )

     uav3_launch_include = IncludeLaunchDescription(
        
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_ros'),'launch/spawn_dron.launch')),

        DeclareLaunchArgument("ns",default_value=TextSubstitution(text="uav3")),#no me cierra el declare acá
        DeclareLaunchArgument("ID",default_value=TextSubstitution(text="3")),
        DeclareLaunchArgument("vehicle",default_value=TextSubstitution(text="$(arg vehicle)")),
        DeclareLaunchArgument("sdf",default_value=TextSubstitution(text="$(arg sdf)")),
        DeclareLaunchArgument("est", default_value=TextSubstitution(text="$(arg est)")),
        DeclareLaunchArgument("rtB",default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("x", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("y", dafault_value=TextSubstitution(text="-1")),
        DeclareLaunchArgument("z", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("R", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("P", dafault_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("Y", dafault_value=TextSubstitution(text="-1.5710")),

   
     )

#carga las coordenadas en el parameter server y en el world de gazebo

     earth_launch_include = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('earth')),
            
            Node(
               parameters=[{
                "latitude": LaunchConfiguration('latitude'),
                "longitude": LaunchConfiguration('longitude'),
                "amsl": LaunchConfiguration('amsl'),
            }]),

            Node(parameters=[{"cmd": command("xmlstarlet ed --inplace \
                -d '//spherical_coordinates' \
                -s '//world' -t elem -n spherical_coordinates \
                -s '//world/spherical_coordinates' -t elem -n surface_model -v EARTH_WGS84 \
                -s '//world/spherical_coordinates' -t elem -n latitude_deg -v $(arg latitude)\
                -s '//world/spherical_coordinates' -t elem -n longitude_deg -v $(arg longitude)\
                -s '//world/spherical_coordinates' -t elem -n elevation -v $(arg amsl)\
                $(arg world)" )}])              
         ]
    )


 
     launch_include = IncludeLaunchDescription(
        
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch/empty_world.launch')), #cambiar a version en pyton

        DeclareLaunchArgument("gui",default_value=TextSubstitution(text="$(arg gui)")),#no me cierra el declare acá
        DeclareLaunchArgument("world_name",default_value=TextSubstitution(text="$(arg world)")),#ver si estan bien pasados lo de arg
        DeclareLaunchArgument("debug",default_value=TextSubstitution(text="$(arg debug)")),
        DeclareLaunchArgument("verbose",default_value=TextSubstitution(text="$(arg verbose)")),
        DeclareLaunchArgument("paused", default_value=TextSubstitution(text="$(arg paused)")),
   
     )



     return LaunchConfiguration([
       latitude_launch_arg, longitude_launch_arg, amsl_launch_arg, world_launch_arg, est_launch_arg, vehicle_launch_arg,sdf_launch_arg, 
       gui_launch_arg, debug_launch_arg, verbose_launch_arg, paused_launch_arg, rviz_launch_arg, rviz_cfg_launch_arg,
       uav0_launch_include, uav1_launch_include, uav2_launch_include, uav3_launch_include, earth_launch_include, launch_include 
     ])




     