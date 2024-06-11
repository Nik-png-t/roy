#include <ros/ros.h>
#include <group_controller.hpp>


int main(int argc, char **argv){
    ros::init(argc, argv, "group_controller_node");
	ros::NodeHandle n;
	ros::Rate rate(60);
	double stack_time = 0.016;
    int number_of_leds = 3;
    geometry_msgs::Point a, b, c, d;
    b.x = 2;c.y = 2; d.x = 2;d.y = 2;
    LeaderGroupFlight leader(n, "0", a);
    vector<geometry_msgs::Point> led_start_positions = {b,c,d};
    vector<LedGroupFlight*> leds;
    for (int i = 0; i < number_of_leds; i++){
        leds.push_back(new LedGroupFlight(n, to_string(i+1), led_start_positions[i]));
    }

	while(ros::ok())
	{
        for (int i = 0; i < number_of_leds; i++){
            (*leds[i]).update(stack_time);
        }
		ros::spinOnce();
        rate.sleep();
	}
    return 0;
}