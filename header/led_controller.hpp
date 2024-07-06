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
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 




using namespace std;

struct PID{
	PID();
	PID(double k_p, double k_i, double k_d, double controlLimit);
	void setDesiredPosition(double desiredPosition_);
	double pid(double currentPosition, double dt);
	double saturation(double inputVal);
	private:
        double error;
        double desiredPosition;
        double error_past;
        double integral;
        double k_p, k_i, k_d, controlLimit;
};



class LedController{
        static const double k_angle_desired, k_angle_leader, k_velocity_leader, pid_positionXY_p, pid_positionXY_i, pid_positionXY_d,
                                pid_rateXY_p, pid_rateXY_i, pid_rateXY_d, pid_positionZ_p, pid_positionZ_i, pid_positionZ_d;
        string name;
        double leader_is_arm;
        mavros_msgs::State   currentState;
        PID pidX, pidY, pidZ, pidpitch, pidroll;
        ros::NodeHandle n;
        ros::Subscriber local_position_sub;
        ros::Subscriber		stateSub;
        ros::ServiceClient 	arming_client, setModeClient, takeoff_client;
	mavros_msgs::SetMode 	setModeName;
        ros::Publisher  position_pub, takeoff_position_pub;
        geometry_msgs::Vector3 leader_velocity;
        geometry_msgs::Point leader_position, local_position;
        mavros_msgs::PositionTarget setPoint;
        geometry_msgs::Quaternion local_q, leader_q;
        geometry_msgs::Point quaternionToAngle(geometry_msgs::Quaternion& q);
        // client
        char* hostname, *port, buffer[256];
        int sockfd, portno;
        struct sockaddr_in serv_addr;
        struct hostent *server;
    public:
        LedController(ros::NodeHandle node, string name, char* hostname, char* port);
        ~LedController();
        void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);
        bool leader_armed();
        bool local_armed();
        bool arm(bool cmd);
        void update();
        void rosNodeInit();
        void setPointTypeInit();
        void stop_client();
        void start_client();
        void receive_message();
};