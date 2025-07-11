#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/Imu.h>


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


class LedGroupFlight{
        double safety_radius;
        string name;
        PID pidX, pidY, pidZ, pidpitch, pidroll;
        ros::NodeHandle n;
        ros::Subscriber leader_position_sub, leader_velocity_sub, local_position_sub, leader_mission_sub, leader_imu_sub, local_velocity_sub, local_imu_sub;
        ros::Publisher  position_pub;
        geometry_msgs::Vector3 leader_velocity, local_velocity;
        geometry_msgs::Point leader_position, start_position, local_position, mission_position;
        mavros_msgs::PositionTarget setPoint;
        geometry_msgs::Quaternion local_q, leader_q;
        geometry_msgs::Vector3 leader_accel, local_accel;
        geometry_msgs::Point quaternionToAngle(geometry_msgs::Quaternion& q);
    public:
        LedGroupFlight(ros::NodeHandle node, string name, geometry_msgs::Point start_position);
        void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void leader_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void leader_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void local_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void mission_callback(const mavros_msgs::WaypointList::ConstPtr& msg);
        void leader_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void update(double dt);
        void rosNodeInit();
        void setPointTypeInit();
};

class LeaderGroupFlight{
        ros::NodeHandle n;
        ros::Publisher position_pub;
        ros::Subscriber local_position_sub;
        geometry_msgs::PoseStamped local_position;
        geometry_msgs::Point start_position;
        string name;
    public:
        LeaderGroupFlight(ros::NodeHandle node, string name_, geometry_msgs::Point start_position);
        void rosNodeInit();
        void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};


