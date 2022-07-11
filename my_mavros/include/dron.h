/************************************************************************
*   Includes
************************************************************************/
//roscpp
#include <ros/ros.h>
#include <ros/duration.h>

//Mensajes
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <mavros_msgs/State.h>

//TF2 related includes
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Geodesy
#include <geodesy/utm.h>
#include <cmath>
#include <string>
#include <GeographicLib/Geoid.hpp>

//Otras clases
//#include "controller.h"

class Dron 
{
private:
    //INFORMACION DEL DRON
    std::string ns;
    ros::NodeHandle nh;
    mavros_msgs::State state;
    geometry_msgs::PoseStamped pose;

    //MISCELANEOS
    ros::Subscriber map_to_odom_sub, odom_to_base_sub, state_sub, pose_sub;
    tf2_ros::StaticTransformBroadcaster static_tf_br;
    tf2_ros::TransformBroadcaster tf_br;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    //CLASES
    //Controller controller;

    //METODOS PRIVADOS
    bool pub_earth_to_map();

    //CALLBACKS
    void map_to_odom_cb (const nav_msgs::Odometry::ConstPtr & map_to_base);
    void odom_to_base_cb (const nav_msgs::Odometry::ConstPtr & msg);
    void state_cb (const mavros_msgs::State::ConstPtr& msg);
    void pose_cb (const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    //CONSTRUCTORES
    Dron(ros::NodeHandle& nh, char* ns);

    //METODOS PUBLICOS
    bool is_connected() const;
    void wait_for_connection() const;
    ros::NodeHandle& get_nh();
    mavros_msgs::State get_state() const;
    geometry_msgs::PoseStamped get_pose() const;
};