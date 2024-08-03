#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <atomic>
#include <thread>


using namespace std;

class LeaderController{
        ros::NodeHandle n;
        ros::Subscriber local_position_sub, local_velocity_sub, stateSub, local_acceleration_sub;

        geometry_msgs::Point local_position;
        geometry_msgs::Vector3 local_velocity, local_acceleration;
        geometry_msgs::Quaternion local_q;
        mavros_msgs::State	currentState;

        vector<thread> pool;
        // server
        int sockfd, portno;
        vector<int> sock_clients;
        char* port;
        socklen_t clilen;
        char buffer[256];
        struct sockaddr_in  serv_addr,  cli_addr;
    public:
        LeaderController(ros::NodeHandle node, char* port);
        ~LeaderController();
        void rosNodeInit();
        void update();
        void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void local_acceleration_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void start_server();
        void stop_server();
        void publish_message(string message);
};