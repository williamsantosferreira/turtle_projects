#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>
#include <vector>


float x,y,w,theta,vel_x;
int contador = 0;

	

void take_pose(const turtlesim::Pose::ConstPtr& pose){
	float dx,dy,theta,e_x,e_y,k,dtheta;
	float x_pos[]={5,8,8,5};
	float y_pos[]={5,5,2,2};

	x = pose->x;
	y = pose->y;
	theta = pose->theta;

	vel_x = pose->linear_velocity;
	w = pose->angular_velocity;


	k = 10;

	dx = x_pos[contador] - pose->x;
	dy = y_pos[contador] - pose->y;

	theta = atan2(dy,dx);
	dtheta = theta - pose->theta;
	
	w = k*dtheta;

	e_x = 0.1;
	e_y = 0.1;


	ROS_INFO("X_robot = %f\nY_robot = %f\nX_desired = %f\nY_desired = %f\n\n",pose->x,pose->y,x_pos[contador],y_pos[contador]);
	
	if((fabs(dx) <= e_x) && (fabs(dy) <= e_y)){
		contador++;
		if(contador == 4)
			contador = 0;
}
		
	else
		vel_x = sqrt(dx*dx + dy*dy);

}



int main(int argc, char** argv){
	ros::init(argc,argv,"turtle_sim_control");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
	ros::Subscriber sub = n.subscribe("turtle1/pose",1000,take_pose);
	ros::Rate freq(10);

	geometry_msgs::Twist vel;

	

	while(ros::ok()){
		
				
		vel.angular.z = w;
		vel.linear.x = vel_x;
		pub.publish(vel);

		
		
		



		freq.sleep();
		ros::spinOnce();
	}
	

	return 0;
}
