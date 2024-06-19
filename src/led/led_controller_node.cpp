#include <ros/ros.h>
#include <led_controller.hpp>


int main(int argc, char *argv[]){
	if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    ros::init(argc, argv, "led_controller_node");
	ros::NodeHandle n;
	ros::Rate rate(60);
	int number_of_drone = 1;
    ros::Time last_request = ros::Time::now();
    LedController led = LedController(n, to_string(number_of_drone), argv[1], argv[2]);

	while(ros::ok())
	{
        
        if((ros::Time::now() - last_request > ros::Duration(5.0)) || led.local_armed() != led.leader_armed()){
            led.arm(led.leader_armed());
            last_request = ros::Time::now();
        }
        led.update();
		ros::spinOnce();
        rate.sleep();
	}
    return 0;
}