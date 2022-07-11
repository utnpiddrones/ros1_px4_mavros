#include <ros/ros.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <stdlib.h>
#include <cmath>

class Dron;

class Controller 
{
private:
    //CONSTANTES
    const int STDIN = 0;
    const double PI = 3.14159265358979323846;
    static const int OFFLINE = 0;
    static const int MANUAL = 1;
    static const int FILE = 2;
    
    //INFO ENTRE THREADS
    Dron* dron;
    std::thread th_setpoint;
    std::thread th_input;
    std::thread th_file;
    geometry_msgs::PoseStamped setpoint; //Posicion a viajar
    bool activated; //Si los threads deben seguir corriendo o no
    int mode;
    char* file_path;

    //MISCELANEOS
    struct termios old_terminal_cfg; //Configuración vieja de la terminal
    ros::Publisher setpoint_pub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    //MIEMBROS PRIVADOS
    void change_terminal_cfg() const;
    void shutdown();

    //THREADS
    void th_setpoint_pub();
    void th_input_handler();
    void th_file_reader();

public:
    //CONSTRUCTORES
    Controller(Dron* dron);
    ~Controller();

    //MIEMBROS PUBLICOS
    bool init(int mode, char* file);
};
