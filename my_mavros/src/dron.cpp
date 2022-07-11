#include "dron.h"

/*=============================================================================================================================================
*   CONSTRUCTORES
===============================================================================================================================================*/
/******************************************************************************
*   Brief:  Constructor. Inicializa el controlador y subscribe a todos los 
*           mensajes con información del dron.
*
*   Input:  nh: el nodeHandler del topic donde se llame.
*           ns: el namespace del topic donde se llame (uav0, uav1, etc).
*
*   Output: None.
******************************************************************************/
Dron::Dron(ros::NodeHandle& nh, char* ns): nh(nh), ns(ns), tf_listener(tf_buffer)
{
    //Subscribo a todos los mensajes relevantes
    state_sub = nh.subscribe("mavros/state", 10, &Dron::state_cb, this);                                //Mensaje de estado.
    map_to_odom_sub = nh.subscribe("mavros/global_position/local", 10, &Dron::map_to_odom_cb, this);    //Posición en el mapa del dron.
    odom_to_base_sub = nh.subscribe("mavros/local_position/odom", 10, &Dron::odom_to_base_cb, this);    //Odometría. Posición desde el frame del dron.
    pose_sub = nh.subscribe("mavros/local_position/pose", 10, &Dron::pose_cb, this);                    //Posicion actual.

    wait_for_connection();

    if ( pub_earth_to_map() == false)
    {
        ROS_ERROR("Error al querer publicar earth -> %s_map.\n", ns );
    }
}

/*=============================================================================================================================================
*   METODOS PUBLICOS
===============================================================================================================================================*/
bool Dron::is_connected() const
{
    return state.connected;
}

void Dron::wait_for_connection() const
{
    ros::Rate loop(10);
    while (!is_connected() && ros::ok() )
    {
        ros::spinOnce();
        loop.sleep();
    }
}

ros::NodeHandle& Dron::get_nh()
{
    return nh;
}

mavros_msgs::State Dron::get_state() const
{
    return state;
}

geometry_msgs::PoseStamped Dron::get_pose() const
{
    return pose;
}

/*=============================================================================================================================================
*   MÉTODOS PRIVADOS
===============================================================================================================================================/
/******************************************************************************
*   Brief:  Publica de forma ESTATICA la tranformacion entre el frame "earth"
*           y "map". Las coordenadas del origen del frame "earth" están definidas
*           en el archivo launch.
*
*   Input:  Void.
*
*   Output: En exito, "true". Si hubo algun error, retorna "false".
******************************************************************************/
bool Dron::pub_earth_to_map()
{
    geometry_msgs::TransformStamped earth_to_map;
    GeographicLib::Geoid geoid("egm96-5");
    
    // global_postion/global tiene las coordenadas GPS desde donde spawneo
    // global_position/local tiene la posicion en UTM (desde donde spawneo) y la orientación.
    sensor_msgs::NavSatFix::ConstPtr gps_msg;
    nav_msgs::Odometry::ConstPtr local_msg;

    //Lee UN solo mensaje de los topics
    if ( (  gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("mavros/global_position/global", this->nh, ros::Duration(60) ) ) == NULL ||
         (  local_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("mavros/global_position/local", this->nh, ros::Duration(60) ) ) == NULL )
    {
        return false;
    }

    //Coordendas en WGS84 de la base
    geographic_msgs::GeoPoint earth_gps;
    nh.getParam("/earth/latitude", earth_gps.latitude);
    nh.getParam("/earth/longitude", earth_gps.longitude);
    nh.getParam("/earth/amsl", earth_gps.altitude);
    earth_gps.altitude += geoid(earth_gps.latitude, earth_gps.longitude); // h = H + N, donde h = alt. del ellipsoide; H = AMSL (alt. ortográfica), N = alt. del geoide

    //Coordenadas en WGS84 de la ubicación de inicio del dron
    geographic_msgs::GeoPoint map_gps;
    map_gps.latitude = gps_msg->latitude;
    map_gps.longitude = gps_msg->longitude;
    map_gps.altitude = gps_msg->altitude;

    //Transformamos de WGS84 a UTM (X, Y, Z) = ENU = (easting, northing, up)
    geodesy::UTMPoint earth_utm(earth_gps);
    geodesy::UTMPoint map_utm(map_gps);

    double distance_x = map_utm.easting - earth_utm.easting;
    double distance_y = map_utm.northing - earth_utm.northing;
    double distance_z = map_utm.altitude - earth_utm.altitude;

    //Cargo todo en la transformada y la envio
    earth_to_map.header.stamp = local_msg->header.stamp;
    earth_to_map.header.frame_id = "earth";
    earth_to_map.child_frame_id = ns + "_map";

    earth_to_map.transform.translation.x = distance_x;
    earth_to_map.transform.translation.y = distance_y;
    earth_to_map.transform.translation.z = distance_z;

    //Se deja sin rotación, para que todos los mapas tengan la misma referencia.
    //earth_to_map.transform.rotation = local_msg->pose.pose.orientation;
    earth_to_map.transform.rotation.x = 0;
    earth_to_map.transform.rotation.y = 0;
    earth_to_map.transform.rotation.z = 0;
    earth_to_map.transform.rotation.w = 1;

    static_tf_br.sendTransform(earth_to_map);

    return true;
}

/*=============================================================================================================================================
*   CALLBACKS
==============================================================================================================================================*/
/******************************************************************************
*   Brief:  Publica la tranformacion entre el frame "map" y "odom". 
*
*   Input:  El contenido del mensaje del topic /mavros/global_position/local,
*           que contiene la posición del robot fusionada con información GPS
*           en el mapa y que puede presentar saltos discontinuos.
*
*   Output: void
******************************************************************************/
void Dron::map_to_odom_cb (const nav_msgs::Odometry::ConstPtr& map_to_base)
{
    geometry_msgs::TransformStamped map_to_odom, base_to_odom;

    tf2::Transform t_base_to_odom, t_map_to_base, t_map_to_odom;

    //Transformada base -> odom
    base_to_odom = tf_buffer.lookupTransform (ns + "_base_link", ns + "_odom", map_to_base->header.stamp, ros::Duration(1.0) ); 

    t_base_to_odom.setOrigin( tf2::Vector3( base_to_odom.transform.translation.x,
                                            base_to_odom.transform.translation.y,
                                            base_to_odom.transform.translation.z) );

    t_base_to_odom.setRotation( tf2::Quaternion(    base_to_odom.transform.rotation.x,
                                                    base_to_odom.transform.rotation.y,
                                                    base_to_odom.transform.rotation.z,
                                                    base_to_odom.transform.rotation.w ) );

    //Transformada base -> map
    t_map_to_base.setOrigin( tf2::Vector3(  map_to_base->pose.pose.position.x,
                                            map_to_base->pose.pose.position.y,
                                            map_to_base->pose.pose.position.z) );

    t_map_to_base.setRotation( tf2::Quaternion( map_to_base->pose.pose.orientation.x,
                                                map_to_base->pose.pose.orientation.y,
                                                map_to_base->pose.pose.orientation.z,
                                                map_to_base->pose.pose.orientation.w ) );

    // map -> odom = map -> base * base -> odom;
    t_map_to_odom = t_map_to_base * t_base_to_odom;

    //Cargamos todo en el mensaje
    map_to_odom.header.stamp = map_to_base->header.stamp;
    map_to_odom.header.frame_id = ns + "_map";
    map_to_odom.child_frame_id = ns + "_odom";

    map_to_odom.transform.translation.x = t_map_to_odom.getOrigin().x();
    map_to_odom.transform.translation.y = t_map_to_odom.getOrigin().y();
    map_to_odom.transform.translation.z = t_map_to_odom.getOrigin().z();

    map_to_odom.transform.rotation.x = t_map_to_odom.getRotation().x();
    map_to_odom.transform.rotation.y = t_map_to_odom.getRotation().y();
    map_to_odom.transform.rotation.z = t_map_to_odom.getRotation().z();
    map_to_odom.transform.rotation.w = t_map_to_odom.getRotation().w();

    tf_br.sendTransform(map_to_odom);
}

/******************************************************************************
*   Brief:  Publica la tranformacion entre el frame "odom" y "base_link"
*
*   Input:  El contenido del mensaje del topic /mavros/local_position/odom,
*           que contiene la odometría continua, sin saltos discretos, referenciada}
*           desde el frame del dron.
*
*   Output: void
******************************************************************************/
void Dron::odom_to_base_cb (const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::TransformStamped odom_to_base;

    //Cargamos los datos de la transformada y la enviamos
    odom_to_base.header.stamp = msg->header.stamp;
    odom_to_base.header.frame_id = ns + "_odom";
    odom_to_base.child_frame_id = ns + "_base_link";

    odom_to_base.transform.translation.x = msg->pose.pose.position.x;
    odom_to_base.transform.translation.y = msg->pose.pose.position.y;
    odom_to_base.transform.translation.z = msg->pose.pose.position.z;

    odom_to_base.transform.rotation = msg->pose.pose.orientation;

    tf_br.sendTransform(odom_to_base);
}


void Dron::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    this->state = *msg;
}

void Dron::pose_cb (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    this->pose = *msg;
}

