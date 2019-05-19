#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>


const int SPIN_RATE=1;



enum spinMode_t {DISABLED, SPININPLACE, DANCE};

class Spin{
	private:
		spinMode_t mode;
		geometry_msgs::Twist base_rate;
		geometry_msgs::Twist prev_spin_cmd;

		int angle;
		int direction;

	public:


		Spin() : mode(DISABLED),angle(0),direction(1){}
		
		void processVel(const geometry_msgs::Twist::ConstPtr& cmdMsg){
			base_rate=*cmdMsg;
		}

		spinMode_t getMode(){return mode;}



		void spinInPlace( ros::Publisher &spin_pub ){
			geometry_msgs::Twist spin;
			geometry_msgs::Vector3 angular;
			angular.x=0;
			angular.y=0;
			angular.z=SPIN_RATE;

			geometry_msgs::Vector3 linear;
			linear.x=0;
			linear.y=0;
			linear.z=0;

			spin.angular=angular;
			spin.linear=linear;

			spin_pub.publish(spin);
		}


		void processCmd(const std_msgs::String::ConstPtr &cmdMsg){
			std::string cmd=cmdMsg->data;
			switch (cmd[0]){
				case 's'://spin
				mode=SPININPLACE;
				break;
				
				case 'd'://dance
				mode=DANCE;
				break;

				default:
				mode=DISABLED;
				break;
			}

		}
};




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "comm");
	ros::NodeHandle n;

	ros::Publisher vel_Pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	Spin Spinny;

	ros::Subscriber vel_Sub = n.subscribe("/cmd_vel",100, &Spin::processVel, &Spinny);

	ros::Subscriber spin_Sub = n.subscribe("/spin",100, &Spin::processCmd, &Spinny);

	

	ros::Rate loop_rate(100);

	
	while (ros::ok())
	{
		switch(Spinny.getMode()){
			case SPININPLACE:
			Spinny.spinInPlace(vel_Pub);
			break;
			case DANCE:
			break;
			default:
			break;

		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}