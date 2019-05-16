#include "ros/ros.h"
#include "std_msgs/String.h"

#include "roborts_msgs/Telemetry.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/ShootInfo.h"

#include "tf2_msgs/TFMessage.h"

#include <sstream>

class Communication{
	private:		 
		 roborts_msgs::Telemetry telemMsg;

	public:
		Communication(){
		 telemMsg.remain_hp=0;
		 telemMsg.mode="";
		 telemMsg.enemy_loc0=tf2_msgs::TFMessage();
		 telemMsg.enemy_loc1=tf2_msgs::TFMessage();
		 telemMsg.remain_bullet=0;
		 telemMsg.self_loc=tf2_msgs::TFMessage();
		}

		void hpCallback(const roborts_msgs::RobotStatus::ConstPtr& statusMsg){
			telemMsg.remain_hp=statusMsg->remain_hp;	
		}

		void modeCallback(const std_msgs::String& modeMsg){
			telemMsg.mode=modeMsg.data;
		}

		void e0Callback(const tf2_msgs::TFMessage::ConstPtr& locMsg){
			telemMsg.enemy_loc0=*locMsg;
		}

		void e1Callback(const tf2_msgs::TFMessage::ConstPtr& locMsg){
			telemMsg.enemy_loc1=*locMsg;
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

	ros::Subscriber hpSub = n.subscribe("/RobotStatus", 100, &Communication::hpCallback, &Comms);
	ros::Subscriber modeSub = n.subscribe("/mode", 100, &Communication::modeCallback, &Comms);
	ros::Subscriber enemy0Sub = n.subscribe("/enemy0", 100, &Communication::e0Callback, &Comms);
	ros::Subscriber enemy1Sub = n.subscribe("/enemy1", 100, &Communication::e1Callback, &Comms);//not sure the actual topic names
	ros::Subscriber ammoSub = n.subscribe("/ShootInfo", 100, &Communication::ammoCallback, &Comms);	
	ros::Subscriber selfSub = n.subscribe("/tf", 100, &Communication::locCallback, &Comms);

	ros::Rate loop_rate(10);


	
	int count = 0;	// A count of how many messages we have sent, used to create a unique string for each message.
	while (ros::ok())
	{

		ROS_INFO("%s", ((std::string)("Remaining hp: "+std::to_string(Comms.getMsg().remain_hp))).c_str());
		ROS_INFO("%s", ((std::string)("mode: "+Comms.getMsg().mode)).c_str());
		ROS_INFO("%s", ((std::string)("Remaining ammo: "+std::to_string(Comms.getMsg().remain_bullet))).c_str());
		ROS_INFO("%s", ((std::string)("msg count: "+std::to_string(count))).c_str());
		ROS_INFO("%s", ((std::string)("ID: "+id)).c_str());

		 //"Enemy0: "+std::to_string(Comms.getMsg().enemy_loc0);
		 //"Enemy1: "+std::to_string(Comms.getMsg().enemy_loc1);
		 //"Self location: "+std::to_string(Comms.getMsg().self_loc);
	
		Comms.publish(bridge_pub);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}

	


