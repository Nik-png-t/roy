#include <leader_controller.hpp>

void error(const char *msg)
{
    perror(msg);
    exit(1);
}



LeaderController::LeaderController(ros::NodeHandle node, char* port) : n(node), port(port){
    rosNodeInit();
    
    pool.emplace_back(&LeaderController::start_server, this);
    
    
}

LeaderController::~LeaderController(){
    stop_server();
}

void LeaderController::rosNodeInit(){
    local_velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local/", 10, &LeaderController::local_velocity_callback, this);
    local_acceleration_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &LeaderController::local_acceleration_callback, this);
    local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &LeaderController::local_position_callback, this);
	stateSub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, &LeaderController::uavStateCallback, this);
}

void LeaderController::uavStateCallback(const mavros_msgs::State::ConstPtr& msg){
	currentState = *msg;
}

void LeaderController::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = msg->pose.position;
    local_q = msg->pose.orientation;
}

void LeaderController::local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_velocity = msg->twist.linear;
}

void LeaderController::local_acceleration_callback(const sensor_msgs::Imu::ConstPtr& msg){
    local_acceleration = msg->linear_acceleration;
}

void LeaderController::update(){
    double siny_cosp = 2 * (local_q.w * local_q.z + local_q.x * local_q.y);
    double cosy_cosp = 1 - 2 * (local_q.y * local_q.y + local_q.z * local_q.z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    string msg = "";
    double v[11] = {currentState.armed, local_position.x, local_position.y, local_position.z,
     local_velocity.x, local_velocity.y, local_velocity.z, local_acceleration.x, local_acceleration.y, local_acceleration.z, yaw};
    for (int i = 0; i < 11; i++){
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
    while (1){
        listen(sockfd,5);
        clilen = sizeof(cli_addr);
        sock_clients.push_back(accept(sockfd, 
                    (struct sockaddr *) &cli_addr, 
                    &clilen));
        if (sock_clients[-1] < 0) 
            error("ERROR on accept");
    }
}

void LeaderController::stop_server(){
    for (int i = 0; i < sock_clients.size(); i++){
        close(sock_clients[i]);
    }
    close(sockfd);
}

void LeaderController::publish_message(string message){
    for (int i = 0; i < sock_clients.size(); i++){
        try{
            cout << "wait a message" << endl;
            bzero(buffer,256);
            int n_ = read(sock_clients[i],buffer,255);
            if (n_ < 0) throw "ERROR reading from socket";
            cout << "send_message" << endl;
            cout << message << endl;
            bzero(buffer, 256);
            n_ = write(sock_clients[i], message.data(), message.size());
            if (n_ < 0) throw "ERROR writing to socket";
        }
        catch(const char *error){
            sock_clients.erase(sock_clients.begin() + i);
        }
    }
}

