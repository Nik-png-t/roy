#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "json/json.h"
#include <fstream>




using namespace std;


class LedController{
        ros::NodeHandle n;

        ros::Subscriber local_position_sub, local_velocity_sub, local_acceleration_sub, stateSub;
        ros::ServiceClient 	arming_client, setModeClient, takeoff_client;
        ros::Publisher  position_pub, takeoff_position_pub;

        mavros_msgs::State   currentState;
	mavros_msgs::SetMode 	setModeName;
        geometry_msgs::Vector3 leader_velocity, local_velocity, leader_acceleration, local_acceleration;
        geometry_msgs::Point leader_position, local_position, error_position, error_velocity, error_acceleration;
        
        mavros_msgs::PositionTarget setPoint;
        double k_position_leader, k_acceleration_leader, k_velocity_leader, leader_is_arm, leader_yaw;
        // client
        char* hostname, *port, buffer[256];
        int sockfd, portno;
        struct sockaddr_in serv_addr;
        struct hostent *server;
    public:
        LedController(ros::NodeHandle node, char* hostname, char* port);
        ~LedController();

        void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void local_acceleration_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

        void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);
        
        void arm(bool cmd);
        void read_data();
        void update();
        void rosNodeInit();
        void setPointTypeInit();

        void stop_client();
        void start_client();
        void receive_message();

        bool leader_armed(){return leader_is_arm;}
        bool local_armed(){return (int)currentState.armed;}
};