#include "ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

void spinInPlace( ros::NodeHandle & nodeHandle ){
	geometry_msgs::Twist spin;
	geometry_msgs::Vector3 angular;
	angular.x=0;
	angular.y=0;
	angular.z=5;
	spin.angular=angular;
	ros::Publisher spin_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	spin_pub.publish(spin);
}

