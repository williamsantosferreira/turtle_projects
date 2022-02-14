#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <std_msgs/String.h>
#include <string>

/*Para verificar se o frame está sendo alterado, no terminal digite:
rosrun tf tf_echo /world /turtle1
E rode o turtlesim movimentando, e ele irá alterar o frame

Podemos também utilizar o RVIZ para verificar esse movimento
*/

std::string turtle_name = "turtle1";


void callback(const turtlesim::Pose::ConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));

	//Obtendo orientação em quaternios
	tf::Quaternion q;
	q.setRPY(0,0,msg->theta); //converte ROLL PITCH YAW para quatérnio
	transform.setRotation(q);
	
	//Transformada final
	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));//world é o world_frame enquanto que turtle_name é o child_frame
	
}

int main(int argc, char** argv){
	ros::init(argc,argv,"tf_broadcaster");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(turtle_name+"/pose",10,&callback);
	
	ros::spin();
	return 0;
}
