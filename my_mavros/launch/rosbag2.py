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
#from drivers.launch.action import SetLaunchConfiguration


def generate_launch_description():
    
    #<!--Seleccione la cantidad de drones a spawnear-->
    ns0_launch_arg = DeclareLaunchArgument("ns0", default_value=TextSubstitution(text="uav0"))
    ns1_launch_arg = DeclareLaunchArgument("ns1", default_value=TextSubstitution(text="uav1"))
    ns2_launch_arg = DeclareLaunchArgument("ns2", default_value=TextSubstitution(text="uav2"))
    ns3_launch_arg = DeclareLaunchArgument("ns3", default_value=TextSubstitution(text="uav3"))

    #<!--La visualización (solo aplica para el primer dron)-->
    
    rviz_launch_argument = DeclareLaunchArgument("rviz", default_value=TextSubstitution("true"))
    rviz_cfg_launch_argument = DeclareLaunchArgument("rviz_cfg", default_value =  TextSubstitution("$(find my_mavros)/scripts/config/cylinder.rviz"))
    rtabmapviz_launch_argument = DeclareLaunchArgument("rtabmapviz", default_value =  TextSubstitution("false"))

    #<!--Los bag-file y data-bases donde guardará la grabación-->

    bag_file_launch_argument = DeclareLaunchArgument("bag_file", default_value =  TextSubstitution("$(find my_mavros)/recordings/cylinder/recording.bag"))

    #<!-- WARNING: LAS DB SE SOBRESCRIBIRÁN-->

    database0_launch_argument = DeclareLaunchArgument("database0", default_value = TextSubstitution("$(find my_mavros)/recordings/default/$(arg ns0).db"))
    database1_launch_argument = DeclareLaunchArgument("database1", default_value = TextSubstitution("$(find my_mavros)/recordings/default/$(arg ns1).db"))
    database2_launch_argument = DeclareLaunchArgument("database2", default_value = TextSubstitution("$(find my_mavros)/recordings/default/$(arg ns2).db"))
    database3_launch_argument = DeclareLaunchArgument("database3", default_value = TextSubstitution("$(find my_mavros)/recordings/default/$(arg ns3).db"))

    #<!--Corremos el rosbag pausado, apriete SPACE para iniciarlo, una vez que se hayan ejecutado todos los programas
     
    player_node = Node(
        package = 'rosbag',
        arguments = '--clock --pause $(arg bag_file)',
        type = 'play',
        name = 'player',
        output = 'screen'
     )


    #inicio de los RTAB-MAP

     # include another launch file in the chatter_ns namespace
    launch_include_ns0 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('$(arg ns0)')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_mavros'),'launch/rtab.launch')),

                DeclareLaunchArgument("rviz",default_value=TextSubstitution(text="$(arg rviz)")),#no me cierra el declare acá
                DeclareLaunchArgument("rviz_cfg",default_value=TextSubstitution(text="$(arg rviz_cfg)")),
                DeclareLaunchArgument("rtabmapviz",default_value=TextSubstitution(text="$(arg rtabmapviz)")),

                DeclareLaunchArgument("ns",default_value=TextSubstitution(text="$(arg ns0)")),
                DeclareLaunchArgument("recording", default_value=TextSubstitution(text="true")),
                DeclareLaunchArgument("database",default_value=TextSubstitution(text="$(arg database0)")),

               # SetLaunchValue
             )                                   
         ]
    )

    launch_include_ns1 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('$(arg ns1)')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_mavros'),'launch/rtab.launch')),

                DeclareLaunchArgument("ns",default_value=TextSubstitution(text="$(arg ns1)")),
                DeclareLaunchArgument("recording", default_value=TextSubstitution(text="true")),
                DeclareLaunchArgument("database",default_value=TextSubstitution(text="$(arg database1)")),

               # SetLaunchValue
             )                                    
         ]
    )

    launch_include_ns2 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('$(arg ns2)')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_mavros'),'launch/rtab.launch')),

                DeclareLaunchArgument("ns",default_value=TextSubstitution(text="$(arg ns2)")),
                DeclareLaunchArgument("recording", default_value=TextSubstitution(text="true")),
                DeclareLaunchArgument("database",default_value=TextSubstitution(text="$(arg database2)")),

               # SetLaunchValue
             )                                    
         ]
    )

    launch_include_ns3 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('$(arg ns3)')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('my_mavros'),'launch/rtab.launch')),

                DeclareLaunchArgument("ns",default_value=TextSubstitution(text="$(arg ns3)")),
                DeclareLaunchArgument("recording", default_value=TextSubstitution(text="true")),
                DeclareLaunchArgument("database",default_value=TextSubstitution(text="$(arg database3)")),

               # SetLaunchValue
             )
         ]
    )



    return LaunchDescription([
        ns0_launch_arg, ns1_launch_arg, ns2_launch_arg, ns3_launch_arg, rviz_cfg_launch_argument, rviz_launch_argument,
        rtabmapviz_launch_argument, database0_launch_argument, database1_launch_argument, database2_launch_argument,
        database3_launch_argument, bag_file_launch_argument, player_node, launch_include_ns0, launch_include_ns1,
        launch_include_ns2, launch_include_ns3
 ])

        
    
   #ver lo de setlaunchconfiguration tratar de meter ahi lo del group.   

