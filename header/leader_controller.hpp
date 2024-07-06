#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/WaypointList.h>
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
 
//std::atomic_int acnt;
 
    



using namespace std;

class LeaderController{
        vector<thread> pool;
        ros::NodeHandle n;
        ros::Subscriber local_position_sub, local_velocity_sub, stateSub;
        geometry_msgs::PoseStamped local_position;
        geometry_msgs::Vector3 local_velocity;
        mavros_msgs::State	currentState;
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
        void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void start_server();
        void stop_server();
        void publish_message(string message);
};