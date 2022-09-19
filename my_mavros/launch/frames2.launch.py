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

from drivers.launch.action import SetLaunchConfiguration



def generate_launch_description():

    # inputs
    ns_launch_arg = DeclareLaunchArgument("ns", default_value=TextSubstitution(text="uav0"))

    #other things
     
    pi_2_launch_arg = SetLaunchConfiguration("pi/2", value=TextSubstitution(text="1.5707963267948966"))#no se si va bien esto)

    #frame estaticos

    tf_base_to_camera_node = Node(
            package='tf2_ros',
            arguments='0 0 0.08 -$(arg pi/2) 0 -$(arg pi/2) $(arg ns)_base_link $(arg ns)_camera_link',
            type='static_transform_publisher', #no existe type
            name='tf_base_to_camera'
        )
    
    tf_base_to_imu_node = Node(
            package='tf2_ros',
            arguments='0 0 0 0 0 0 $(arg ns)_base_link $(arg ns)_imu_link', #ch
            type='static_transform_publisher',
            name='tf_base_to_imu'
        )


    return LaunchDescription([
    ns_launch_arg, pi_2_launch_arg, tf_base_to_camera_node, tf_base_to_imu_node,
        
 ])

# <!--Frame estático declarado en el tree frame-->
#<!--Cámara apuntando hacia las +x   0 0 0 -1.57 0 -1.57   -->




