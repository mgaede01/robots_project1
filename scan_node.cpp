#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>
#include <stdlib.h>
#include <stdio.h>

//Global Velocity Message Publisher
ros::Publisher vel_cmd;

//Global Velocity Message
geometry_msgs::Twist tw;

//PI constant
double PI = 3.141592;

/*
 * Function
 * Generate radians from a degree angle
 * Formula Radian = 180/PI Degrees
 */
double generateAngular(int angle){
	return (double)((rand() % (angle*2)) - angle)/180.0*PI;
}

/*
 * Function:
 * Drive forward for approximately one foot
 */
void layer5_drive() {

	//Set rate of robot to 10 Hz
	ros::Rate r(10);

	//Set Robot to move 1 foot in 1 second
	tw.linear.x = 0.33;
	tw.angular.z = 0;

	//Loop for 1 second publishing velocy message
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(1.0);
	while(ros::Time::now() - start_time < timeout) {
		vel_cmd.publish(tw);
		r.sleep();
	}
}

/*
 * Function:
 * Drive normally and turn approximately +-15 degrees
 */
void layer4_turn() {

	//Set rate of robot to 10 Hz
	ros::Rate r(10);

	//Set Robot to turn +-15 degrees in 1 second
	tw.linear.x = 0.33;
	tw.angular.z = generateAngular(15);

	//Loop for 1 second publishing velocy message
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(1.0);
	while(ros::Time::now() - start_time < timeout) {
		vel_cmd.publish(tw);
		r.sleep();
	}
}

/*
 * Function:
 * Stop immediately and turn approximately 180+-30 degrees.
 */
void layer3_escape(int direction) {

	//Set rate of robot to 10 Hz
	ros::Rate r(10);

	//Set Robot to stop driving
	tw.linear.x = 0;

	//Set Robot to turn depending on direction of
	//closest scan 180+-30 degrees
	if(direction > 0) {
		tw.angular.z = PI+generateAngular(30);
	} else {
		tw.angular.z = -PI-generateAngular(30);
	}

	//Loop for 1 second publishing velocy message
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(1.0);
	while(ros::Time::now() - start_time < timeout) {
		vel_cmd.publish(tw);
		r.sleep();
	}

	//Loop for 1 second publishing zero velocy message to stop turning
	start_time = ros::Time::now();
	ros::Duration one_sec(1.0);
	while(ros::Time::now() - start_time < one_sec) {
		vel_cmd.publish(geometry_msgs::Twist());
		r.sleep();
	}
}

/*
 * Function:
 * Callback function for each time a scan is taken for processing
 */
void layers_laser(const sensor_msgs::LaserScan::ConstPtr& msg){

		//Declare variables to obtain lowest value
		float lowest = 999.99;
		int lowest_index = -1;

		//Loop to check every 80th message.
		//Still gives multiple scans to read from for this case.
		for(int i = 0; i < msg->ranges.size(); i += 79){

			//Grab lowest
			if(msg->ranges[i]<lowest){
				lowest = msg->ranges[i];
				lowest_index = i;
			}
		}

		//If lowest is outside of range, turn away from that object
		if(lowest <= 1) {
			if(lowest_index < (msg->ranges.size()/2)){
				layer3_escape(1);
			} else {
				layer3_escape(-1);
			}

		//If not, then drive forward
		} else {
			layer4_turn();
			layer5_drive();
		}

		//Publish center lazer scan for debugging
		ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);
}
/////////////////////////////////////////////////////////////////////////////////////////
//TODO document this function
/////////////////////////////////////////////////////////////////////////////////////////

void userDrive(){
	ros::Rate r(10);
	char c;
	std::cout << "Drive robot by using 'a' for left, 'd' for right, 'w' for forward,\n's' for backward, 'x' for autonomous robot control";
	while (std::cin.get(c)) {
		tw.linear.x = 0;
		tw.angular.z = 0;
    if(c == 'w') {
			tw.linear.x = 1;
		} else if(c == 's') {
			tw.linear.x = -1;
		} else if(c == 'a') {
			tw.angular.z = 1;
		} else if(c == 'd') {
			tw.angular.z = -1;
		} else {
			if(c == 'x'){
				return;
			}
		}
		vel_cmd.publish(tw);
		r.sleep();
	}
}

/*
 * Function:
 * Halt immediately if robot hits something
 */
void layer1_bumper(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
	ros::Rate r(10);
	vel_cmd.publish(geometry_msgs::Twist());
	r.sleep();
	exit(EXIT_FAILURE);
}

/*
 * Function:
 * Main driver of the robots program
 */
int main(int argc, char **argv){

	//Initialize the node to control the robot
	ros::init(argc, argv, "scan_node");
	ros:: NodeHandle n;

	//Subscribe to the bumperevent topic
	ros::Subscriber sub_bumper = n.subscribe("/mobile_base/events/bumper", 1, layer1_bumper);

	//Subscribe to lazer scan topic
	ros::Subscriber sub_laser = n.subscribe("/scan", 1, layers_laser);

	//Publish to Command Velocity Topic
	vel_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

	//Let user drive robot to starting location
	userDrive();

	//While ros is doing fine, go ahead and spin once
	while(ros::ok()) {
		ros::spinOnce();
	}
}
