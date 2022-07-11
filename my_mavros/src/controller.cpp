#include "dron.h"

//DEPRECATED. ANDA, PERO NO SE COMPILA (Es necesario agregar el include en dron.h)

/*=============================================================================================================================================
*   CONSTRUCTOR Y DESTRUCTOR
===============================================================================================================================================/
/******************************************************************************
*   Brief:  Constructor sin funcionalidad.
*
*   Input:  dron: Un puntero al dron que tiene este controlador.
*           activate: si activar el controlador o no.
*
*   Output: None.
******************************************************************************/
Controller::Controller(Dron* dron): dron(dron), activated(false), mode(OFFLINE)
{
}

/******************************************************************************
*   Brief:  Si fue activado, libera los recursos (reinicia el terminal y espera
*           por los threads).
******************************************************************************/
Controller::~Controller()
{
    if(activated)
    {
        shutdown();
    } 
}

/*=============================================================================================================================================
*   MIEMBROS PÚBLICOS
===============================================================================================================================================/
/******************************************************************************
*   Brief:  Inicializa la operación del controlador. Carga la variable
*           "activated", indicando que se empezó a controlar.
*
*   Input:  Modos de operación, a saber:
*           Controller::OFFLINE = 0
*           Controller::MANUAL = 1
*           Controller::FILE = 2
*
*   Output: true si pudo iniciarse con éxito, false si hubo error.
******************************************************************************/
bool Controller::init(int mode, char* file)
{
    if (dron->is_connected())
    {
        this->mode = mode;
        this->file_path = file;

        switch (mode)
        {
            case OFFLINE:
            {
                break;
            }

            case MANUAL:
            {
                activated = true;
                //Publicar la posición del dron
                setpoint_pub = dron->get_nh().advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
                //Preparar el dron
                arming_client = dron->get_nh().serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
                set_mode_client = dron->get_nh().serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

                tcgetattr(STDIN, &(this->old_terminal_cfg));

                change_terminal_cfg();
                
                //Inicializo los threads
                th_input = std::thread(&Controller::th_input_handler, this);
                th_setpoint = std::thread(&Controller::th_setpoint_pub, this);

                break;
            }

            case FILE:
            {
                activated = true;
                //Publicar la posición del dron
                setpoint_pub = dron->get_nh().advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
                //Preparar el dron
                arming_client = dron->get_nh().serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
                set_mode_client = dron->get_nh().serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

                th_file = std::thread(&Controller::th_file_reader, this);
                th_setpoint = std::thread(&Controller::th_setpoint_pub, this);
                break;
            }

            default:
            {
                break;
            }
        }

        return true;
    }

    else
    {
        return false;
    }
}

/*=============================================================================================================================================
*   MIEMBROS PRIVADOS
===============================================================================================================================================/
/******************************************************************************
*   Brief:  Cambia la configuración de la terminal para que no sea buffereada.
*           El input se recibirá apenas se escriba, sin apretar enter.
*
*   Input:  Void.
*
*   Output: Void.
******************************************************************************/
void Controller::change_terminal_cfg() const
{
    struct termios current;
    current = this->old_terminal_cfg;
    current.c_lflag &= ~ICANON;
    current.c_lflag &= ~ECHO;
    tcsetattr(STDIN, TCSANOW, &current);
}

/******************************************************************************
*   Brief:  Limpia todo lo creado por la clase, sin destruirla.
*
*   Input:  Void.
*
*   Output: Void.
******************************************************************************/
void Controller::shutdown()
{
    activated = false; //Orden para que salgan todos los threads.
    setpoint_pub.shutdown();
    arming_client.shutdown();
    set_mode_client.shutdown();
    if (mode = MANUAL)
    {
        tcsetattr(STDIN, TCSANOW, &(this->old_terminal_cfg));
        th_input.join();
    }

    else if (mode = FILE)
    {
        th_file.join();
    }
    
    th_setpoint.join();
}

/*=============================================================================================================================================
*   THREADS
===============================================================================================================================================/
/******************************************************************************
*   Brief:  Thread encargado de publicar los setpoints para que avanze el dron.
*
*   Input:  None.
*
*   Output: None.
******************************************************************************/
void Controller::th_setpoint_pub()
{
    mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

    ros::Rate rate(20);

    ros::Time last_request;

    setpoint = dron->get_pose();
    
    //send a few setpoints before starting
    for(int i = 0; ros::ok() && i < 100; i++)
    {
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    last_request = ros::Time::now();

    while(ros::ok() && activated)
    {
        if( dron->get_state().mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_ERROR("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !dron->get_state().armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
}

/******************************************************************************
*   Brief:  Thread encargado de recibir el input del usuario, y procesarlo en
*           posiciones del nuevo setpoint para que avanze el dron.
*           TODO: este thread no me deja escribir con ROS_INFO.
*
*   Input:  Void.
*
*   Output: Void.
******************************************************************************/
void Controller::th_input_handler()
{
    double rpy[3] = {0,0,0};
    tf2::Quaternion quat;
    //tf2::Matrix3x3 rotation;

    setpoint = dron->get_pose();
    //TODO: inicializar el dron con una orientación distinta de rpy={0,0,0}
    //tf2::convert(setpoint.pose.orientation, quat);
    //rotation.setRotation(quat);
    //rotation.getRPY(rpy[0], rpy[1], rpy[2]);

    ROS_ERROR("\nInstrucciones de vuelo\nWASD: movimiento.\nQ,E: rotacion.\nSPACEBAR, Z: ascender y descender.\nF: Salir\n");

    while(activated && ros::ok() )
    {
        switch (std::cin.get())
        {
            case 'w':
            {
                setpoint.pose.position.x += 0.1*std::cos(rpy[2]);
                setpoint.pose.position.y += 0.1*std::sin(rpy[2]);
                break;
            }
            case 'a':
            {
                setpoint.pose.position.x += 0.1*std::cos(rpy[2] + PI/2);
                setpoint.pose.position.y += 0.1*std::sin(rpy[2] + PI/2);
                break;
            }
            case 's':
            {
                setpoint.pose.position.x -= 0.1*std::cos(rpy[2]);
                setpoint.pose.position.y -= 0.1*std::sin(rpy[2]);
                break;
            }
            case 'd':
            {
                setpoint.pose.position.x -= 0.1*std::cos(rpy[2] + PI/2);
                setpoint.pose.position.y -= 0.1*std::sin(rpy[2] + PI/2);
                break;
            }

            case ' ':
            {
                setpoint.pose.position.z += 0.1;
                break;
            }

            case 'z':
            {
                setpoint.pose.position.z -= 0.1;
                break;
            }
            case 'q':
            {
                rpy[2] += PI/64;
                quat.setRPY(rpy[0], rpy[1], rpy[2]);
                tf2::convert(quat, setpoint.pose.orientation);
                break;
            }
            case 'e':
            {
                rpy[2] -= PI/64;
                quat.setRPY(rpy[0], rpy[1], rpy[2]);
                tf2::convert(quat, setpoint.pose.orientation);
                break;
            }

            case 'f':
            {
                this->activated = false;
                break;
            }

            default:
            {
                break;
            }
        }

    }
}

/******************************************************************************
*   Brief:  Lee el archivo con las misiones, y las ejecuta.
*           El formato del archivo deben ser:
*           x y z R P Y
*
*   Input:  Void.
*
*   Output: Void.
******************************************************************************/
void Controller::th_file_reader()
{
    double rpy[3];
    tf2::Quaternion quat;
    std::fstream file (file_path, std::ios::in);
    char line[100]; //Tamaño de seguridad
    int index = 0;
    ros::Rate rate(10.0);

    sleep(30);

    if (file.fail())
    {
        ROS_ERROR("No se pudo abrir el archivo con la misión\n");
        activated = false;
    }

    while (ros::ok() && activated)
    {
        index = 0;
        file.getline(line, 30);
        if (file.eof())
        {
            activated = false;
        }

        else
        {
            ROS_ERROR("ORIGINAL SETPOINT %f", setpoint.pose.position.x);
            setpoint.pose.position.x = atof(&line[index]);
            for (++index; index < 100 && line[index] != ' '; index++);
            setpoint.pose.position.y = atof(&line[++index]);
            for (++index; index < 100 && line[index] != ' '; index++);
            setpoint.pose.position.z = atof(&line[++index]);
            for (++index; index < 100 && line[index] != ' '; index++);
            rpy[0] = atof(&line[++index]);
            for (++index; index < 100 && line[index] != ' '; index++);
            rpy[1] = atof(&line[++index]);
            for (++index; index < 100 && line[index] != ' '; index++);
            rpy[2] = atof(&line[++index]);
            quat.setRPY(rpy[0], rpy[1], rpy[2]);
            quat.normalize();
            tf2::convert(quat, setpoint.pose.orientation);

            while( std::sqrt(std::pow(setpoint.pose.position.x - dron->get_pose().pose.position.x, 2) + std::pow(setpoint.pose.position.y - dron->get_pose().pose.position.y, 2) + std::pow(setpoint.pose.position.z - dron->get_pose().pose.position.z, 2) ) > 0.1 )
            {
                rate.sleep();
            }
        }

        ROS_ERROR("SETPOINT ENVIADO %f", setpoint.pose.position.x);

        //Hasta que la distancia entre el setpoint y la posicion actual sea mayor a 0.1m, no leas el proximo punto de la mision
        

        ROS_ERROR("ENVIE MISION");
        
    }
}