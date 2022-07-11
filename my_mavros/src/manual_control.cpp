/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <iostream>
#include <string>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void cb_user_input (geometry_msgs::PoseStamped& pose, bool& quit)
{
    char aux;
    float rpy[3] = {0,0,0};
    tf2::Quaternion quat;
    ROS_INFO ("INICIO THREAD\n");
    ros::Rate loop(20);
    do
    {
        fflush(stdout);
        std::cin.get(aux);
        if (std::cin.fail() || std::cin.eof())
        {
            std::cin.clear();
        }
        else
        {
            switch (aux)
            {
                case 'w':
                {
                    pose.pose.position.x += 0.1;
                    break;
                }
                case 'a':
                {
                    pose.pose.position.y += 0.1;
                    break;
                }
                case 's':
                {
                    pose.pose.position.x -= 0.1;
                    break;
                }
                case 'd':
                {
                    pose.pose.position.y -= 0.1;
                    break;
                }
                case 'q':
                {
                    rpy[2] += 0.1;
                    quat.setRPY(rpy[0], rpy[1], rpy[2]);
                    tf2::convert(quat, pose.pose.orientation);
                    break;
                }
                case 'e':
                {
                    rpy[2] -= 0.1;
                    quat.setRPY(rpy[0], rpy[1], rpy[2]);
                    tf2::convert(quat, pose.pose.orientation);
                    break;
                }

                default:
                {
                    break;
                }
            }
        }
        loop.sleep();

    }while(!quit);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3.2;
    bool quit = false;
    std::thread thread(cb_user_input, std::ref(pose), std::ref(quit));

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }



    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

    }
    quit = true;
    thread.join();

    return 0;
}

