
//DEPRECATED De momento, no uso este archivo, pero setea un nuevo global_position/origin

#include "ros/ros.h"
#include "geographic_msgs/GeoPointStamped.h"

#define LATITUDE -34.660019
#define LONGITUDE -58.468302
#define ALTITUDE 30.18 //14m sobre el nivel del mar convertido a ellipsoidal

void publish_gp_origin (ros::NodeHandle & nh)
{
    int exit = 0;

    ros::Publisher pub = nh.advertise<geographic_msgs::GeoPointStamped>("uav0/mavros/global_position/set_gp_origin", 5);
    //Creación de la variable mensaje e inicialización de variables
    geographic_msgs::GeoPointStamped gp_origin;
    gp_origin.header.frame_id = "map";
    gp_origin.position.latitude = LATITUDE;
    gp_origin.position.longitude = LONGITUDE;
    gp_origin.position.altitude = ALTITUDE;

    ROS_INFO ("===============================\n> Publicando el origen de coordenadas del frame earth (en LLA)\n===============================\n");
    
    ros::Rate loop(2); //Loop de 2Hz
    
    while (ros::ok() && exit < 20) 
    {
        pub.publish (gp_origin); //Publico el mensaje
        ros::spinOnce();
        loop.sleep();
        exit++;
    }
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "main_node");
    ros::NodeHandle nh;

    publish_gp_origin (nh);
   
    return 0;
}