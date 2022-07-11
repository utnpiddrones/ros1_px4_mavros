#include "dron.h"
#include "string.h"
#include <stdlib.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drones"); 
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0); //Multi-thread
  bool control = false;
  if (argc < 2)
  {
      ROS_ERROR("El dron no pudo ejecutarse por un problema en el roslaunch\n");
  }
  else
  {
      Dron dron(nh, argv[1]);  
      spinner.start();
      ros::waitForShutdown();
  }

  return 0;
};