#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>

ros::Publisher vel_cmd;
geometry_msgs::Twist tw;


void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg){
	float lowest = 999.99;
	int lowest_index = -1;

	ros::Rate r(10); // 10 hz
	tw.linear.x = .7;
	tw.angular.z = 0;

	for(int i = 0; i < msg->ranges.size(); i += 79){
		if(msg->ranges[i]<lowest){
			lowest = msg->ranges[i];
			lowest_index = i;
		}
	}
	if(lowest <= 1.0) {
		tw.linear.x = 0;
		if(lowest_index < (msg->ranges.size()/2)){
			tw.angular.z = 1.1;
		} else {
			tw.angular.z = -.9;
		}
		ros::Time start_time = ros::Time::now();
		ros::Duration timeout(4.0); // Timeout of 2 seconds
		while(ros::Time::now() - start_time < timeout) {
  		vel_cmd.publish(tw);
			r.sleep();
		}
		start_time = ros::Time::now();
		ros::Duration one_sec(1.0); // Timeout of 2 seconds
		while(ros::Time::now() - start_time < one_sec) {
			vel_cmd.publish(geometry_msgs::Twist());
			r.sleep();
		}
	} else {
		vel_cmd.publish(tw);
	}
	r.sleep();

	ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "scan_node");
	ros:: NodeHandle n;
	ros::Subscriber sub_laser = n.subscribe("/scan", 1, clbk_laser);
	vel_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	ros::spin();
}
