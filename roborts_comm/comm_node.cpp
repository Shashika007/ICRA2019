#include "ros/ros.h"
#include "std_msgs/String.h"

#include "roborts_msgs/Telemetry.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/ShootInfo.h"

#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

class Communication{
	private:		 
		 roborts_msgs::Telemetry telemMsg;

	public:
		Communication(){
		 telemMsg.remain_hp=0;
		 telemMsg.mode="";
		 telemMsg.enemy_loc=geometry_msgs::PoseStamped();
		 telemMsg.remain_bullet=0;
		 telemMsg.self_loc=tf2_msgs::TFMessage();
		}

		void hpCallback(const roborts_msgs::RobotStatus::ConstPtr& statusMsg){
			telemMsg.remain_hp=statusMsg->remain_hp;	
		}

		void modeCallback(const std_msgs::String& modeMsg){
			telemMsg.mode=modeMsg.data;
		}

		void enemyCallback(const geometry_msgs::PoseStamped::ConstPtr& locMsg){
			telemMsg.enemy_loc=*locMsg;
		}


		void ammoCallback(const roborts_msgs::ShootInfo::ConstPtr& locMsg){
			telemMsg.remain_bullet=locMsg->remain_bullet;
		}

		void locCallback(const tf2_msgs::TFMessage::ConstPtr& locMsg){
			telemMsg.self_loc=*locMsg;
		}

		 roborts_msgs::Telemetry getMsg(){
			 return telemMsg;
		 }


		void publish(const ros::Publisher& robo_pub){
			robo_pub.publish(telemMsg);
		}
};




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "comm");
	ros::NodeHandle n;


	std::string id;
	n.getParam("/id",id);

	ros::Publisher bridge_pub = n.advertise<roborts_msgs::Telemetry>("/comm/robo"+id, 100);

	Communication Comms;

	ros::Subscriber hpSub 		= n.subscribe("/RobotStatus", 100, &Communication::hpCallback	, &Comms);
	ros::Subscriber modeSub 	= n.subscribe("/mode"		, 100, &Communication::modeCallback	, &Comms);
	ros::Subscriber enemySub 	= n.subscribe("/enemy_pose"	, 100, &Communication::enemyCallback, &Comms);
	ros::Subscriber ammoSub 	= n.subscribe("/ShootInfo"	, 100, &Communication::ammoCallback	, &Comms);	
	ros::Subscriber selfSub 	= n.subscribe("/tf"			, 100, &Communication::locCallback	, &Comms);

	ros::Rate loop_rate(10);

	
	int count = 0;	// A count of how many messages we have sent, used to create a unique string for each message.
	while (ros::ok())
	{
		std::string info= std::to_string(count) +" "+ id +" "+ Comms.getMsg().mode +" hp: "+ std::to_string(Comms.getMsg().remain_hp) +" ammo: "+ std::to_string(Comms.getMsg().remain_bullet);

		ROS_INFO("%s", info.c_str());
	
		Comms.publish(bridge_pub);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}

	


