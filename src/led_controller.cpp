#include <led_controller.hpp>

LedController::LedController(ros::NodeHandle node, char* hostname, char* port) : n(node), hostname(hostname), port(port){
    leader_is_arm = false;
    read_data();
    rosNodeInit();
    start_client();
    cout << "starting server!" << endl;
    setPointTypeInit();
    
}

LedController::~LedController(){
    stop_client();
}

void LedController::rosNodeInit(){
    local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &LedController::local_position_callback, this);
    local_velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &LedController::local_velocity_callback, this);
    local_acceleration_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &LedController::local_acceleration_callback, this);
    position_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1, this);
    setModeClient = n.serviceClient<mavros_msgs::SetMode>( "/mavros/set_mode");
	arming_client = n.serviceClient<mavros_msgs::CommandBool>( "/mavros/cmd/arming");
	stateSub = n.subscribe<mavros_msgs::State>( "/mavros/state", 10, &LedController::uavStateCallback, this);
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>( "/mavros/cmd/takeoff");
    takeoff_position_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1, this);
}

void LedController::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = msg->pose.position;
}

void LedController::local_acceleration_callback(const sensor_msgs::Imu::ConstPtr& msg){
    local_acceleration = msg->linear_acceleration;
}

void LedController::local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_velocity = msg->twist.linear;
}

void LedController::uavStateCallback(const mavros_msgs::State::ConstPtr& msg){
	currentState = *msg;
}

void LedController::update(){
    cout << "update" << endl;
    receive_message();

    error_velocity.x = leader_velocity.x-local_velocity.x;
    error_velocity.y = leader_velocity.y-local_velocity.y;
    error_velocity.z = leader_velocity.z-local_velocity.z;

    error_position.x = leader_position.x-local_position.x;
    error_position.y = leader_position.y-local_position.y;
    error_position.z = leader_position.z-local_position.z;

    error_acceleration.x = leader_acceleration.x - local_acceleration.x;
    error_acceleration.y = leader_acceleration.y - local_acceleration.y;
    error_acceleration.z = leader_acceleration.z - local_acceleration.z;

    setPoint.velocity.x =  leader_velocity.x + error_velocity.x*k_velocity_leader;
	setPoint.velocity.y =  leader_velocity.y + error_velocity.y*k_velocity_leader;
    setPoint.velocity.z =  leader_velocity.z + error_velocity.z;

    setPoint.position.x = leader_position.x + error_position.x*k_position_leader;
    setPoint.position.y = leader_position.y + error_position.y*k_position_leader;
    setPoint.position.z = leader_position.z + error_position.z;

    setPoint.acceleration_or_force.x = leader_acceleration.x + error_acceleration.x*k_acceleration_leader;
    setPoint.acceleration_or_force.y = leader_acceleration.y + error_acceleration.y*k_acceleration_leader;
    setPoint.acceleration_or_force.z = 0;//leader_accel.z;
    setPoint.yaw = leader_yaw;
    
    position_pub.publish(setPoint);
}


void 	LedController::setPointTypeInit()
{
	unsigned int setpointTypeMask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	unsigned int setpointCoordinateFrame = 1;

	setPoint.type_mask = setpointTypeMask;
	setPoint.coordinate_frame = setpointCoordinateFrame;
} 


void LedController::start_client(){
    while (1){
        try{
            sleep(1);
            portno = atoi(port);
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0){
                ROS_ERROR("ERROR opening socket");
                continue;
            }
            server = gethostbyname(hostname);
            if (server == NULL) {
                fprintf(stderr,"ERROR, no such host\n");
                continue;
            }
            bzero((char *) &serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            bcopy((char *)server->h_addr, 
                (char *)&serv_addr.sin_addr.s_addr,
                server->h_length);
            serv_addr.sin_port = htons(portno);
            if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
                ROS_ERROR("ERROR connecting");
                continue;
                }
            break;
        }
        catch(...){
            ROS_ERROR("Server connection failed");
        }
    }
}

void LedController::stop_client(){
    close(sockfd);
}

void LedController::arm(bool cmd){
    if (cmd){
        setModeName.request.custom_mode = "GUIDED";
    }
    else{
        setModeName.request.custom_mode = "LAND";
    }

	mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = cmd;
    if( setModeClient.call(setModeName) &&
        setModeName.response.mode_sent){
        if (cmd)ROS_INFO("Offboard enabled");
        else ROS_INFO("Auto Land enabled");
    }
    
    if(arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ros::spinOnce();
        if (cmd){
            ROS_INFO("Vehicle armed");
            geometry_msgs::PoseStamped msg;
            msg.pose.position.z = 1;
            for (int i = 0; i < 100; i++){
                ros::spinOnce();
                ros::Duration(0.01).sleep();
                takeoff_position_pub.publish(msg);
            }
            mavros_msgs::CommandTOL srv_takeoff;
            srv_takeoff.request.altitude = 1;
            if(takeoff_client.call(srv_takeoff)){
                sleep(6);
                ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
            }else{
                ROS_ERROR("Failed Takeoff");
            }
        }
        else ROS_INFO("Vehicle disarmed");
    }
}




void LedController::receive_message(){
    try{
        cout << sockfd << endl;
        bzero(buffer, 256);
        string buf = "send message to me";
        int n = write(sockfd,buf.data(),buf.size());
        if (n < 0)throw "ERROR writing to socket";
        bzero(buffer,256);
        n = read(sockfd,buffer,255);
        if (n < 0)throw "ERROR reading from socket";
    }
    catch (...){
        start_client();
    }
    try{
        string buf = "";
        int count = 0;
        double *v[11] = {&leader_is_arm, &leader_position.x, &leader_position.y, &leader_position.z, 
                        &leader_velocity.x, &leader_velocity.y, &leader_velocity.z,
                        &leader_acceleration.x, &leader_acceleration.y, &leader_acceleration.z, &leader_yaw};
        for (int i = 0; i < 11;){
            if (buffer[count] == ' '){
                *v[i] = stod(buf);
                buf = "";
                i++;
            }
            buf += buffer[count];
            count++;
        }
    }
    catch(...){
        ROS_ERROR("Failed to process message");
    }
}

void LedController::read_data(){
    Json::Reader reader;  //for reading the data
    Json::Value newValue; //for modifying and storing new values
    Json::StyledStreamWriter writer; //for writing in json files
 
    //opening file using fstream
    ifstream file("../data/setup.json");
 
    // check if there is any error is getting data from the json file
    if (!reader.parse(file, newValue)) {
        cout << reader.getFormattedErrorMessages();
        exit(1);
    }
    Json::FastWriter fastWriter;
    string a[3] = {"удержание позиции","реакция", "удержание скорости полета"};
    double *b[3] = {&k_position_leader, &k_acceleration_leader, &k_velocity_leader};
    for (int i = 0; i < 3; i++){
        *b[i] = stof(fastWriter.write((newValue[a[i]])));
    }
    file.close();
}