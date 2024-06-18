#include <ros/ros.h>
#include <leader_controller.hpp>


int main(int argc, char *argv[]){
	if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
    }
    ros::init(argc, argv, "leader_controller_node");
	ros::NodeHandle n;
	ros::Rate rate(60);
    LeaderController leader(n, "0", argv[1]);

	while(ros::ok())
	{
        leader.update();
		ros::spinOnce();
        rate.sleep();
	}
    return 0;
}