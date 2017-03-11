#include <ackermann_msgs/AckermannDrive.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

//Defintion of constants.
float MAX_ACCELERATING_VOLTAGE;
float MAX_BRAKING_VOLTAGE;
float MAX_STEERING_ANGLE;
float MAX_VELOCITY;
float MIN_SHUTDOWN_VELOCITY;
float VELOCITY_STEP;
int QUEUE_LENGTH;
float WHEEL_DIAMETER;
std::string NOTSTOP_TOPIC;
std::string STEERING_TOPIC;
std::string STELLGROESSEN_TOPIC;
std::string WHEEL_SENSOR_LEFT_TOPIC;
std::string WHEEL_SENSOR_RIGHT_TOPIC;
//Subcriber and publisher.
ros::Publisher analog_acc_pub;
ros::Publisher notstop_pub;
ros::Publisher stellgroessen_pub;
ros::Publisher velocity_pub;
ros::Subscriber ps_controller_sub;
ros::Subscriber steering_angle_sub;
ros::Subscriber wheel_sensor_left_sub;
ros::Subscriber wheel_sensor_right_sub;
//Controller and car state.
double current_angle = 0.0;
double should_angle = 0.0;
bool mode = true;
bool mode_changable = true;
bool notstop = false;
bool shutdown = false;
double current_velocity = 0.0;
double should_velocity = 0.0;
//Declaration of functions.
double absVal(double value);
void closePsControl();
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg);
void initPsControl(ros::NodeHandle* node);
void steeringAngleCallback(const std_msgs::Float64::ConstPtr& msg);
void wheelSensorCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char** argv){
	//Init ros node.
	ros::init(argc, argv, "ps_control");
	ros::NodeHandle node;
	//Getting parameters.
	node.getParam("/erod/MAX_ACCELERATING_VOLTAGE", MAX_ACCELERATING_VOLTAGE);
	node.getParam("/erod/MAX_BRAKING_VOLTAGE", MAX_BRAKING_VOLTAGE);
	node.getParam("/erod/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
	node.getParam("/erod/MAX_VELOCITY", MAX_VELOCITY);
	node.getParam("/erod/WHEEL_DIAMETER", WHEEL_DIAMETER);
	node.getParam("/safety/MIN_SHUTDOWN_VELOCITY", MIN_SHUTDOWN_VELOCITY);
	node.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
	node.getParam("/topic/NOTSTOP", NOTSTOP_TOPIC);
 	node.getParam("/topic/STATE_STEERING_ANGLE", STEERING_TOPIC);
	node.getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_TOPIC);
	node.getParam("/topic/WHEEL_REAR_LEFT", WHEEL_SENSOR_LEFT_TOPIC);
	node.getParam("/topic/WHEEL_REAR_RIGHT", WHEEL_SENSOR_RIGHT_TOPIC);
	VELOCITY_STEP = MAX_VELOCITY/40;
	//Initialisation.
	initPsControl(&node);
	//Spinning.
	ros::spin();
	closePsControl();
	return 0;
}

double absVal(double value){
	if(value >= 0.0) return value;
	if (value < 0.0) return -value;
}

void initPsControl(ros::NodeHandle* node){
	//Publisher and subscriber
	analog_acc_pub = node->advertise<std_msgs::Float64>("/analog_acc", QUEUE_LENGTH);
	notstop_pub = node->advertise<std_msgs::Bool>(NOTSTOP_TOPIC, QUEUE_LENGTH);
	stellgroessen_pub = node->advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_TOPIC, QUEUE_LENGTH);
	velocity_pub = node->advertise<std_msgs::Float64>("/current_velocity", QUEUE_LENGTH);
	ps_controller_sub = node->subscribe("/joy", QUEUE_LENGTH, controllerCallback);
	steering_angle_sub = node->subscribe(STEERING_TOPIC, QUEUE_LENGTH, steeringAngleCallback);
	wheel_sensor_left_sub = node->subscribe(WHEEL_SENSOR_LEFT_TOPIC, QUEUE_LENGTH, wheelSensorCallback);
	wheel_sensor_right_sub = node->subscribe(WHEEL_SENSOR_RIGHT_TOPIC, QUEUE_LENGTH, wheelSensorCallback);
	//Sending initialised comment.
	std::cout << std::endl << "ARC VIEWER: Ps controller initialised "<< std::endl;
}

void closePsControl(){
	std::cout << std::endl << "ARC VIEWER: Ps controller closing ..." << std::endl;
	ros::shutdown();
	ros::waitForShutdown();
}

void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg){
	//Change mode -> X Taste [buttons 2].
	//0 --> progressiv and 1 --> exact.
	if(msg->buttons[2] == 1 && mode_changable){
		mode = !mode;
		mode_changable = false;
		std::cout << std::endl << "ARC VIEWER: Mode changed" << std::endl;
	}
	if(msg->buttons[2] == 0 && !mode_changable) mode_changable = true;
	//Emergency stop -> B Taste [buttons 1].
	if(msg->buttons[1] == 1){
		std_msgs::Bool notstop_msg;
		notstop_msg.data = true;
		notstop_pub.publish(notstop_msg);
		std::cout << std::endl << "ARC VIEWER: NOTSTOP !!!!!" << std::endl;
		while(current_velocity > 0.0){
			should_velocity = 0.0;
		}
		closePsControl();
	}
	//Shutdown -> Y Taste [button 3].
	if(msg->buttons[3] == 1){
		std::cout << std::endl << "ARC VIEWER: Shutdown initialised" << std::endl;
		while(current_velocity >= MIN_SHUTDOWN_VELOCITY){
			should_velocity = 0.0; //TODO: import shutdown function from CAS.
		}
		closePsControl();
	}
	//Updating velocity (Accelerating -> RT, Breaking -> LT).
	double key_position_acc = msg->axes[5];
	double key_position_break = msg->axes[2];
	if(!mode && should_velocity <= MAX_VELOCITY && should_velocity >= 0){
		should_velocity += VELOCITY_STEP*absVal(key_position_acc-1)/2;
		should_velocity -= VELOCITY_STEP*absVal(key_position_break-1)/2;
	} 
	if(mode){
		should_velocity = MAX_VELOCITY*absVal(key_position_acc-1)/2;
	} 
	// Updating analog input.
	float key_analog = msg->axes[4];
	if(key_analog == 0){
		std_msgs::Float64 analog_msg;
		analog_msg.data = 0;
		analog_acc_pub.publish(analog_msg);
	}
	if(key_analog > 0){
		std_msgs::Float64 analog_msg;
		analog_msg.data = key_analog * MAX_ACCELERATING_VOLTAGE;
		analog_acc_pub.publish(analog_msg);
	}
	if(key_analog < 0){
		std_msgs::Float64 analog_msg;
		analog_msg.data = key_analog * MAX_BRAKING_VOLTAGE;
		analog_acc_pub.publish(analog_msg);
	}
	//Updating steering angle.
	double key_position_angle = msg->axes[0];
	should_angle = MAX_STEERING_ANGLE*key_position_angle;
	//Update and publish.
	if(should_velocity < 0) should_velocity = 0;
	if(should_velocity > MAX_VELOCITY) should_velocity = MAX_VELOCITY;
	//Publish.
	ackermann_msgs::AckermannDrive drive_msg; 
	drive_msg.speed = should_velocity;
	drive_msg.steering_angle = should_angle;
	stellgroessen_pub.publish(drive_msg);
}

void steeringAngleCallback(const std_msgs::Float64::ConstPtr& msg){
	current_angle = msg->data;
}

void wheelSensorCallback(const std_msgs::Float64::ConstPtr& msg){
	current_velocity = msg->data * WHEEL_DIAMETER/2;
	std_msgs::Float64 velocity_msg;
	velocity_msg.data = current_velocity;
	velocity_pub.publish(velocity_msg);
}
