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
from launch.drivers.launch.conditions import if_condition

#No modificar

def generate_launch_description():
    arg_ns = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="uav0"))
    #Recording
    arg_recording = DeclareLaunchArgument("recording", default_value=TextSubstitution(text="false"))
    arg_db = DeclareLaunchArgument("database", default_value=os.path.join(get_package_share_directory('my_mavros'),'/recordings/default/$(arg ns).db'))
    #Visualizacion
    arg_rviz = DeclareLaunchArgument("rviz", default_value=TextSubstitution(text="false"))
    arg_rviz_cfg = DeclareLaunchArgument("rviz_cfg", default_value=os.path.join(get_package_share_directory('my_mavros'),'/scripts/config/cylinder.rviz'))
    arg_rtabmapviz = DeclareLaunchArgument("rtabmapviz", default_value=TextSubstitution(text="false"))
    #Inicio el rtabmap
    IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('rtabmap_ros'),'/launch/rtabmap.launch')),
                    #Elijo la visualizacion 
                    DeclareLaunchArgument("rtabmapviz", value=LaunchConfiguration(text="$(arg rtabmapviz)")),
                    DeclareLaunchArgument("rviz", value=LaunchConfiguration(text="$(arg rviz)")),
                    DeclareLaunchArgument((if_condition(LaunchConfiguration('$(arg rviz)'))),"rviz_cfg", 
                        value=LaunchConfiguration(text="$(arg rviz_cfg)")),
                    # Sin que saque datos de odometría por la cámara o por lidar
                    DeclareLaunchArgument("visual_odometry", value=LaunchConfiguration(text="false")),
                    DeclareLaunchArgument("icp_odometry", value=LaunchConfiguration(text="false")),
                    #Sin datos de mapeo previos
                    DeclareLaunchArgument("args", value=LaunchConfiguration(text="-d")),
                    DeclareLaunchArgument("database_path", value=LaunchConfiguration(text="$(arg database)")),
                    DeclareLaunchArgument("use_sim_time", value=LaunchConfiguration(text="$(arg recording)")),
                    # Nombre de los frames en el arbol TF
                    DeclareLaunchArgument("map_frame_id", value=LaunchConfiguration(text="$(arg ns)_map")),
                    DeclareLaunchArgument("odom_frame_id", value=LaunchConfiguration(text="$(arg ns)_odom")),
                    DeclareLaunchArgument("frame_id", value=LaunchConfiguration(text="$(arg ns)_base_link")),
                    # Que no publica nada en el arbol TF, ya me ocupo yo
                    DeclareLaunchArgument("publish_tf_map", value=LaunchConfiguration(text="false")),
                    DeclareLaunchArgument("publish_tf_odom", value=LaunchConfiguration(text="false")),
                    #Topics donde publican los datos la cámara
                    DeclareLaunchArgument("rgb_topic", value=LaunchConfiguration(text="/$(arg ns)/camera/color/image_raw")),
                    DeclareLaunchArgument("depth_topic", value=LaunchConfiguration(text="/$(arg ns)/camera/depth/image_raw")),
                    DeclareLaunchArgument("camera_info_topic", value=LaunchConfiguration(text="/$(arg ns)/camera/color/camera_info1")),
                    DeclareLaunchArgument("depth_camera_info_topic", value=LaunchConfiguration(text="/$(arg ns)/camera/depth/camera_info")),
            )

    return LaunchDescription([arg_ns, arg_rviz, arg_db, arg_recording, arg_rtabmapviz, arg_rviz_cfg])