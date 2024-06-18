#include <leader_controller.hpp>

void error(const char *msg)
{
    perror(msg);
    exit(1);
}



LeaderController::LeaderController(ros::NodeHandle node, string name, char* port) : n(node), name(name), port(port){
    rosNodeInit();
    start_server();
    
}

LeaderController::~LeaderController(){
    stop_server();
}

void LeaderController::rosNodeInit(){
    local_velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("/uav"+name+"/mavros/local_position/velocity_local/", 10, &LeaderController::local_velocity_callback, this);
    local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/uav"+name+"/mavros/local_position/pose", 10, &LeaderController::local_position_callback, this);
}

void LeaderController::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}

void LeaderController::local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_velocity = msg->twist.linear;
}

void LeaderController::update(){
    string msg = "";
    double v[10] = {local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z,
     local_position.pose.orientation.x, local_position.pose.orientation.y, local_position.pose.orientation.z, local_position.pose.orientation.w,
     local_velocity.x, local_velocity.y, local_velocity.z};
    for (int i = 0; i < 10; i++){
        msg += to_string(v[i]) + " ";
    }
    publish_message(msg);
}

void LeaderController::start_server(){
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = atoi(port);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
    if (newsockfd < 0) 
          error("ERROR on accept");
}

void LeaderController::stop_server(){
    close(newsockfd);
    close(sockfd);
}

void LeaderController::publish_message(string message){
    bzero(buffer,256);
    //int n_ = read(newsockfd,buffer,1023);
    //if (n_ < 0) error("ERROR reading from socket");
    int n_ = write(newsockfd, message.data(), message.size());
    if (n_ < 0) error("ERROR writing to socket");
}

