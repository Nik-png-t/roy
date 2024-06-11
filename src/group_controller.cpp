#include <group_controller.hpp>

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





LedGroupFlight::LedGroupFlight(ros::NodeHandle node, string name, geometry_msgs::Point start_position) : n(node), name(name), start_position(start_position){
    rosNodeInit();
    setPointTypeInit();
    this->pidX = PID(0.95, 0, 0, 1000); // 30 0 0
	this->pidY = PID(0.95, 0, 0, 1000);  // 30 0 0
	this->pidZ = PID(6.5, 0, 0, 1000);  // 1.6 0.3 0.5
	this->pidroll = PID(1, 0.003, 0.2, 100); // 1 0 0
	this->pidpitch = PID(1, 0.003, 0.2, 100); 

    safety_radius = sqrt(pow(start_position.x, 2) + pow(start_position.y, 2) + pow(start_position.z, 2));
}

void LedGroupFlight::rosNodeInit(){
    leader_velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/local_position/velocity_local/", 5, &LedGroupFlight::leader_velocity_callback, this);
    leader_position_sub = n.subscribe<geometry_msgs::PoseStamped>("leader_position", 5, &LedGroupFlight::leader_position_callback, this);
    local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/uav"+name+ "/mavros/local_position/pose", 10, &LedGroupFlight::local_position_callback, this);
    position_pub = n.advertise<mavros_msgs::PositionTarget>("/uav"+name+"/mavros/setpoint_raw/local", 1, this);
}

void LedGroupFlight::leader_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    leader_position = msg->pose.position;
}

void LedGroupFlight::leader_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    leader_velocity = msg->twist.linear;
}

void LedGroupFlight::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = msg->pose.position;
    local_q = msg->pose.orientation;
}
void LedGroupFlight::update(double dt){
    double siny_cosp = 2 * (local_q.w * local_q.z + local_q.x * local_q.y);
    double cosy_cosp = 1 - 2 * (local_q.y * local_q.y + local_q.z * local_q.z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    // geometry_msgs::Point difference;
    // difference.x = leader_position.x - local_position.x + start_position.x;
    // difference.y = leader_position.y - local_position.y + start_position.y;
    // difference.z = leader_position.z - local_position.z + start_position.z;
    // double r = sqrt(pow(difference.x, 2) + pow(difference.y, 2) + pow(difference.z, 2));
    // cout << "r = " << r << endl;
    // double k = r > safety_radius ? r/safety_radius : safety_radius/r;
    
    // cout << name << endl;
    // cout << local_position.x << " " << local_position.y << " " << local_position.z << endl;
    // cout << k << endl;
    // setPoint.position.x = local_position.x + difference.x;
    // setPoint.position.y = local_position.y + difference.y;
    // setPoint.position.z = local_position.z + difference.z;
    // cout << setPoint.position.x << ", " << setPoint.position.y << ", " << setPoint.position.z << endl;
    
    double sinr_cosp = 2 * (local_q.w * local_q.x + local_q.y * local_q.z);
    double cosr_cosp = 1 - 2 * (local_q.x * local_q.x + local_q.y * local_q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (local_q.w * local_q.y - local_q.x * local_q.z));
    double cosp = std::sqrt(1 - 2 * (local_q.w * local_q.y - local_q.x * local_q.z));
    double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
	

	// set position
	pidX.setDesiredPosition(leader_position.x); 
	pidY.setDesiredPosition(leader_position.y);
	pidZ.setDesiredPosition(leader_position.z);

	// PID control angle
	double pitch_desired = pidX.pid(local_position.x, dt);
	double roll_desired = pidY.pid(local_position.y, dt);
	double velocityZ = pidZ.pid(local_position.z, dt);

	// set angle
	pidpitch.setDesiredPosition(pitch_desired);
	pidroll.setDesiredPosition(roll_desired);

	// PID control velocity
	double velocityX = pidpitch.pid(pitch*cos(yaw) + roll*sin(yaw), dt);
	double velocityY = pidroll.pid(roll*cos(yaw) - pitch*sin(yaw), dt);

	// set
    cout << velocityX << " " << (leader_velocity.x*cos(yaw) + leader_velocity.y*sin(yaw)) << endl;
	setPoint.velocity.x = velocityX + leader_velocity.x;//(leader_velocity.x*cos(yaw) + leader_velocity.y*sin(yaw));
	setPoint.velocity.y = velocityY + leader_velocity.y;//(leader_velocity.y*cos(yaw) - leader_velocity.x*sin(yaw));
	setPoint.velocity.z = velocityZ + leader_velocity.z;//leader_velocity.z;
    //setPoint.velocity.x = ;
    //setPoint.velocity.y = ;
    //setPoint.velocity.z = ;
    position_pub.publish(setPoint);



}



void 	LedGroupFlight::setPointTypeInit()
{
	// задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
	// приведенная ниже конфигурация соответствует управлению линейной скоростью ЛА
	// и угловой скоростью аппарата в канале рыскания(yaw)
	unsigned int setpointTypeMask = mavros_msgs::PositionTarget::IGNORE_PX  +
									mavros_msgs::PositionTarget::IGNORE_PY  +
									mavros_msgs::PositionTarget::IGNORE_PZ  +
									mavros_msgs::PositionTarget::IGNORE_AFX +
									mavros_msgs::PositionTarget::IGNORE_AFY +
									mavros_msgs::PositionTarget::IGNORE_AFZ +
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE +
									mavros_msgs::PositionTarget::IGNORE_YAW;
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



















LeaderGroupFlight::LeaderGroupFlight(ros::NodeHandle node, string name, geometry_msgs::Point start_position) : n(node), name(name), start_position(start_position){
    rosNodeInit();
}

void LeaderGroupFlight::rosNodeInit(){
    cout << name << endl;
    local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("/uav"+name+"/mavros/local_position/pose", 10, &LeaderGroupFlight::local_position_callback, this);
    position_pub = n.advertise<geometry_msgs::PoseStamped>("leader_position", 10, this);
}

void LeaderGroupFlight::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position.pose.position.x = msg->pose.position.x + start_position.x;
    local_position.pose.position.y = msg->pose.position.y + start_position.y;
    local_position.pose.position.z = msg->pose.position.z + start_position.z;
    position_pub.publish(local_position);
}










