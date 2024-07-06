#include <led_controller.hpp>

void error(const char *msg)
{
    perror(msg);
    exit(0);
}


PID::PID(){};
PID::PID(double k_p, double k_i, double k_d, double controlLimit) : k_p(k_p), k_i(k_i), k_d(k_d), controlLimit(controlLimit){
        double error = 0;
        double desiredPosition = 0;
        double error_past = 0;
        double integral = 0;

    }
void PID::setDesiredPosition(double desiredPosition_){
        this->desiredPosition = desiredPosition_;
    }
double PID::pid(double currentPosition, double dt){
        this->error = this->desiredPosition - currentPosition;
        this->integral += this->error * dt;
        double u = this->k_p * this->error + this->k_i * this->integral + this->k_d * ((this->error - this->error_past) / dt);
        this->error_past = this->error;
        return saturation(u);
    }
double PID::saturation(double inputVal){
        if (inputVal > this->controlLimit){
            inputVal = this->controlLimit;
        }
        if (inputVal < -this->controlLimit){
            inputVal = -this->controlLimit;
        }
        return inputVal;
}


LedController::LedController(ros::NodeHandle node, string name, char* hostname, char* port) : n(node), name(name), hostname(hostname), port(port){
    read_data();
    leader_is_arm = false;
    rosNodeInit();
    start_client();
    cout << "startin server!" << endl;
    setPointTypeInit();
    this->pidX = PID(pid_positionX_p, pid_positionX_i, pid_positionX_d, 1000); 
	this->pidY = PID(pid_positionY_p, pid_positionY_i, pid_positionY_d, 1000);  
	this->pidZ = PID(pid_positionZ_p, pid_positionZ_i, pid_positionZ_d, 1000);  
	this->pidroll = PID(pid_rateY_p, pid_rateY_i, pid_rateY_d, 100); 
	this->pidpitch = PID(pid_rateX_p, pid_rateX_i, pid_rateX_d, 100);
    
}

LedController::~LedController(){
    stop_client();
}

void LedController::uavStateCallback(const mavros_msgs::State::ConstPtr& msg){
	currentState = *msg;
}

void LedController::rosNodeInit(){
    local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/uav"+name+ "/mavros/local_position/pose", 1, &LedController::local_position_callback, this);
    position_pub = n.advertise<mavros_msgs::PositionTarget>("/uav"+name+"/mavros/setpoint_raw/local", 1, this);
    setModeClient = n.serviceClient<mavros_msgs::SetMode>("/uav"+name+ "/mavros/set_mode");
	arming_client = n.serviceClient<mavros_msgs::CommandBool>("/uav"+name+ "/mavros/cmd/arming");
	stateSub = n.subscribe<mavros_msgs::State>("/uav"+name+ "/mavros/state", 10, &LedController::uavStateCallback, this);
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/uav"+name+ "/mavros/cmd/takeoff");
    takeoff_position_pub = n.advertise<geometry_msgs::PoseStamped>("/uav"+name+"/mavros/setpoint_position/local", 1, this);
}

void LedController::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = msg->pose.position;
    local_q = msg->pose.orientation;
}

void LedController::update(){
    double dt = 0.16;
    cout << "update" << endl;
    receive_message();
    geometry_msgs::Point local_angle = quaternionToAngle(local_q);
    geometry_msgs::Point leader_angle = quaternionToAngle(leader_q);
	// set position
	pidX.setDesiredPosition(leader_position.x); 
	pidY.setDesiredPosition(leader_position.y);
	pidZ.setDesiredPosition(leader_position.z);

	// PID control angle
	double pitch_desired = pidX.pid(local_position.x, dt);
	double roll_desired = pidY.pid(local_position.y, dt);
	double velocityZ = pidZ.pid(local_position.z, dt);
    // set angle
	pidpitch.setDesiredPosition((leader_angle.x*cos(leader_angle.z) + leader_angle.y*sin(leader_angle.z))*k_angle_leader + pitch_desired*k_angle_desired);
	pidroll.setDesiredPosition((-leader_angle.y*cos(leader_angle.z) + leader_angle.x*sin(leader_angle.z))*k_angle_leader + roll_desired*k_angle_desired);

	// PID control velocity
	double velocityX = pidpitch.pid(local_angle.x*cos(local_angle.z) + local_angle.y*sin(local_angle.z), dt);
	double velocityY = pidroll.pid(-local_angle.y*cos(local_angle.z) + local_angle.x*sin(local_angle.z), dt);

    // set velocity

	setPoint.velocity.x = velocityX + leader_velocity.x*k_velocity_leader;
	setPoint.velocity.y = velocityY + leader_velocity.y*k_velocity_leader;
    setPoint.velocity.z = velocityZ + leader_velocity.z*k_velocity_leader;
    setPoint.yaw = leader_angle.z;
    cout << setPoint.velocity.x <<  " " << setPoint.velocity.y << " " << setPoint.velocity.z << endl;
    position_pub.publish(setPoint);
}

geometry_msgs::Point    LedController::quaternionToAngle(geometry_msgs::Quaternion& q){
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    geometry_msgs::Point a;
    a.x = pitch; a.y = roll; a.z = yaw;
    return a;
}

void 	LedController::setPointTypeInit()
{
	unsigned int setpointTypeMask = mavros_msgs::PositionTarget::IGNORE_AFX  +
									mavros_msgs::PositionTarget::IGNORE_AFY  +
									mavros_msgs::PositionTarget::IGNORE_AFZ  +
									mavros_msgs::PositionTarget::IGNORE_PX +
									mavros_msgs::PositionTarget::IGNORE_PY +
									mavros_msgs::PositionTarget::IGNORE_PZ +
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
									//mavros_msgs::PositionTarget::IGNORE_YAW;//AF, P, V
	// при помощи конфигурации вышеприведенным образом переменной setpointTypeMask
	// можно настроить управление аппаратом посредством передачи(положения аппарата и углового положения в канале рыскания, )

	// конфигурация системы координат в соответствии с которой задаются параметры управления ЛА
	// при setpointCoordinateFrame = 1 управление происходит в неподвижной СК (локальная неподвижная СК инициализируемая при работе навигационной системы)
	// при импользовании ГНСС или optical flow является стартовой, при использовании других НС начало координат соответствует таковому у выбранной
	//  навигационной системы(например оси выходят из центра реперного маркера).
	// setpointCoordinateFrame = 8 соответствует управлению аппаратом в связных нормальных осях (подвижная СК центр которой находится в центре масс ЛА)
	// в действительности без какой либо настройки, совпадает с системой координат инерциальной навигационной системы.
	unsigned int setpointCoordinateFrame = 1;

	// присваевам наши параметры задающего воздействия полям класса нашего сообщения
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

bool LedController::arm(bool cmd){
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
                sleep(10);
                ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
            }else{
                ROS_ERROR("Failed Takeoff");
            }
        }
        else ROS_INFO("Vehicle disarmed");
    }
}


bool LedController::leader_armed(){
    return leader_is_arm;
}

bool LedController::local_armed(){
    return (int)currentState.armed;
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
                        &leader_q.x, &leader_q.y, &leader_q.z, &leader_q.w,
                        &leader_velocity.x, &leader_velocity.y, &leader_velocity.z};
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
        ROS_INFO("Failed to process message");
    }
}

void LedController::read_data(){
    Json::Reader reader;  //for reading the data
    Json::Value newValue; //for modifying and storing new values
    Json::StyledStreamWriter writer; //for writing in json files
    ofstream newFile;
 
    //opening file using fstream
    ifstream file("data/setup.json");
 
    // check if there is any error is getting data from the json file
    if (!reader.parse(file, newValue)) {
        cout << reader.getFormattedErrorMessages();
        exit(1);
    }
    string a[19] = {"k_angle_desired | удержание позиции","k_angle_leader | реакция",
        "k_velocity_leader | удержание скорости полета","pid_positionX_p","pid_positionX_i",
        "pid_positionX_d","pid_rateX_p","pid_rateX_i","pid_rateX_d","pid_positionY_p","pid_positionY_i",
        "pid_positionY_d","pid_rateY_p","pid_rateY_i","pid_rateY_d","pid_positionZ_p","pid_positionZ_i",
        "pid_positionZ_d"};
    double *b[19] = {&k_angle_desired, &k_angle_leader, &k_velocity_leader, &pid_positionX_p, &pid_positionX_i, &pid_positionX_d,
                                &pid_rateX_p, &pid_rateX_i, &pid_rateX_d, &pid_positionY_p, &pid_positionY_i, &pid_positionY_d,
                                &pid_rateY_p, &pid_rateY_i, &pid_rateY_d, &pid_positionZ_p, &pid_positionZ_i, &pid_positionZ_d};
    for (int i = 0; i < 19; i++){
        *b[i] = stof(newValue[a[i]].toStyledString());
    }
}