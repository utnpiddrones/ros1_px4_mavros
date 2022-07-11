#include <iostream>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#define ERROR -1
#define INT_MAX 400000

int main (void)
{
    int input;
    const std::string s_path = "/home/user/catkin_my_mavros/src/my_mavros/scripts/";

    std::string scripts[][2]={  {"Ejecutar simulación.\n",                  s_path + "exe.sh"},
                                {"Imprimir PDF del árbol de frames.\n",     s_path + "print_frames.sh"},
                                {"Grabar un archivo rosbag.\n",             s_path + "record.sh"},
                                {"Correr una instancia de RVIZ.\n",         s_path + "launch_rviz.sh"},
                                {"Correr una grabación.\n",                 s_path + "play_recording.sh"},
                                {"Unir bases de datos.\n",                  s_path + "mergedb.sh"},
                                {"Visualizar base de datos.\n",             s_path + "view_db.sh"},
                                {"Ejecutar QGC\n",                          s_path + "run_qgc.sh"},
                                {"Test.\n",                                 s_path + "test.sh"},
                            };

    int scripts_size = (int)(sizeof(scripts)/sizeof(scripts[0]));

    std::cout << "\nSeleccione una opción\n";

    for (int i = 0; i < scripts_size; i++)
    {  
        scripts[i][0] = std::to_string(i+1) + ".\t" + scripts[i][0];
        std::cout << scripts[i][0];
    }

    do
    {
        if ( !(std::cin >> input) )
        {
            std::cin.clear();
            std::cin.ignore(INT_MAX, '\n');
            std::cout << "Input inválida\n";
            input = ERROR;
        }
        else if (input < 1 || input > scripts_size )
        {   
            std::cout << "Input inválida\n";
            input = ERROR;
        }  
        
    }while(input == ERROR);

    input--;

    system(("chmod +x " + scripts[input][1]).c_str());
    execl(scripts[input][1].c_str(), "", (char *)NULL);


}