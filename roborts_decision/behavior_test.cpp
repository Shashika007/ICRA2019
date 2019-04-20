#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "behaviors/back_boot_area_behavior.h"
#include "behaviors/escape_behavior.h"
#include "behaviors/chase_behavior.h"
#include "behaviors/search_behavior.h"
#include "behaviors/patrol_behavior.h"
#include "behaviors/goal_behavior.h"
#include "behaviors/supply_behavior.h"
#include "behaviors/buff_behavior.h"

#include "blackboard/blackboard.h"



void Command();
char command = '0';


/****** NEW SEQUENCE TYPE DECISION TREE *******/

#include <iostream>
#include <list>

enum Commands {NONE, CHASE, ESCAPE, PATROL, GETBUFF, GETSUPPLY, SEARCH, GOAL, EXIT};
enum TreeType {NONE, DEFAULT_HEAD, SUPPLY_HEAD, HEALTH_HEAD, BUFF_HEAD, ENEMY_HEAD, BONUS_HEAD};

TreeType currentTreeType = NONE;

struct RobotStatus {
	double supply; 			//Percent Supply Left
	double health; 			//Percent Health Left
	bool buffed;   			//Buff status
	bool canGetBuff = false;		//Buff area active to take
	bool lastSeen_inRange;	//Enemy in range.
	
	Commands command;
	
	
	double supplyCutOff = 20;
	double supplyCutOff2 = 40;
	double healthCutOff = 40;
};

class Node {  // This class represents each node in the behaviour tree.
	public:
		virtual bool run() = 0;
};

class CompositeNode : public Node {  //  This type of Node follows the Composite Pattern, containing a list of other Nodes.
	private:
		std::list<Node*> children;
	public:
		const std::list<Node*>& getChildren() const {return children;}
		void addChild (Node* child) {children.emplace_back(child);}
};

class Selector : public CompositeNode {
	public:
		virtual bool run() override {
			std::cout << "NODE: SELECTOR RUNNING" << std::endl;
			
			for (Node* child : getChildren()) {  // The generic Selector implementation
				if (child->run())  // If one child succeeds, the entire operation run() succeeds.  Failure only results if all children fail.
					return true;
			}
			return false;  // All children failed so the entire run() operation fails.
		}
};

class Sequence : public CompositeNode {
	public:
		virtual bool run() override {
			std::cout << "NODE: SEQUENCE RUNNING" << std::endl;

			for (Node* child : getChildren()) {  // The generic Sequence implementation.
				if (!child->run())  // If one child fails, then enter operation run() fails.  Success only results if all children succeed.
					return false;
			}
			return true;  // All children suceeded, so the entire run() operation succeeds.
		}
};

class Inverter : public CompositeNode {
	public:
		virtual bool run() override {
			std::cout << "NODE: INVERTER RUNNING" << std::endl;

			for (Node* child : getChildren()) {  // Inverts single child.				
				return !child->run();
			}
			return true;  // default?
		}	
};



class CheckSupplyStatus : public Node {  // Each task will be a class (derived from Node of course).
	private:
		RobotStatus* status;
		bool boolInv;
	public:
		CheckSupplyStatus (RobotStatus* status, bool inv) : status(status), boolInv(inv) {}
		virtual bool run() override {
			std::cout << "NODE: CHECK_SUPPLY_STATUS" << std::endl;
			if (status->supply > status->supplyCutOff){
				std::cout << "Supply amount is sufficient." << std::endl;  // will return true
				switch(currentTreeType){
					case NONE:
					status->command = NONE;
					break;
					case DEFAULT_HEAD:			
					status->command = NONE;
					break;					
					case SUPPLY_HEAD:
					status->command = NONE;
					break;
					case HEALTH_HEAD:
					status->command = NONE;					
					break;
					case BUFF_HEAD:
					status->command = GETBUFF;
					break;
					case ENEMY_HEAD:			
					status->command = SEARCH;
					break;
					case BONUS_HEAD:
					status->command = NONE;
					break;					
				}				
			}
			else{
				std::cout << "Supply amount is insufficient." << std::endl;  // will return false
				if(boolInv){
					//status->command = GETSUPPLY;	
					switch(currentTreeType){
						case NONE:
						status->command = NONE;
						break;
						case DEFAULT_HEAD:			
						status->command = NONE;
						break;					
						case SUPPLY_HEAD:
						status->command = NONE;
						break;
						case HEALTH_HEAD:
						status->command = NONE;					
						break;
						case BUFF_HEAD:
						status->command = NONE;
						break;
						case ENEMY_HEAD:			
						status->command = GETSUPPLY;
						break;
						case BONUS_HEAD:
						status->command = NONE;
						break;					
					}					
				}
			}
			return (status->supply > status->supplyCutOff);
		}
};

class CheckBuffStatus : public Node {  // Each task will be a class (derived from Node of course).
	private:
		RobotStatus* status;
		bool boolInv;
	public:
		CheckBuffStatus (RobotStatus* status, bool inv) : status(status), boolInv(inv)  {}
		virtual bool run() override {
			std::cout << "NODE: CHECK_BUFF_STATUS" << std::endl;
			if (status->buffed){
				std::cout << "Buff is active." << std::endl;  // will return true
				switch(currentTreeType){
					case NONE:
					status->command = NONE;
					break;
					case DEFAULT_HEAD:			
					status->command = NONE;
					break;					
					case SUPPLY_HEAD:
					status->command = NONE;
					break;
					case HEALTH_HEAD:
					status->command = NONE;					
					break;
					case BUFF_HEAD:
					status->command = NONE;
					break;
					case ENEMY_HEAD:			
					status->command = NONE;
					break;
					case BONUS_HEAD:
					status->command = NONE;
					break;					
				}					
			}
			else{
				std::cout << "Buff is inactive." << std::endl;  // will return false
				if(boolInv){
					//status->command = GETBUFF;
					switch(currentTreeType){
						case NONE:
						status->command = NONE;
						break;
						case DEFAULT_HEAD:			
						status->command = NONE;
						break;					
						case SUPPLY_HEAD:
						status->command = NONE;
						break;
						case HEALTH_HEAD:
						status->command = NONE;					
						break;
						case BUFF_HEAD:
						status->command = NONE;
						break;
						case ENEMY_HEAD:			
						status->command = NONE;
						break;
						case BONUS_HEAD:
						status->command = NONE;
						break;					
					}										
				}
			}
			
			return (status->buffed);
		}
};

class CheckLastSeen : public Node {  // Each task will be a class (derived from Node of course).
	private:
		RobotStatus* status;
		bool boolInv;
	public:
		CheckLastSeen (RobotStatus* status, bool inv) : status(status), boolInv(inv) {}
		virtual bool run() override {
			std::cout << "NODE: CHECK_LASTSEEN_STATUS" << std::endl;

			if (status->lastSeen_inRange){
				std::cout << "Last seen enemy in range." << std::endl;  // will return true
				switch(currentTreeType){
					case NONE:
					status->command = NONE;
					break;
					case DEFAULT_HEAD:			
					status->command = NONE;
					break;					
					case SUPPLY_HEAD:
					status->command = NONE;
					break;
					case HEALTH_HEAD:
					status->command = ESCAPE;					
					break;
					case BUFF_HEAD:
					status->command = GETBUFF;
					if(status->health <= status->healthCutOff)
						status->command = ESCAPE;
					break;
					case ENEMY_HEAD:			
					status->command = NONE;
					break;
					case BONUS_HEAD:
					status->command = NONE;
					break;					
				}					
			}
			else{
				std::cout << "Last seen enemy out of range." << std::endl;  // will return false
				if(boolInv){
					//status->command = PATROL;
					switch(currentTreeType){
						case NONE:
						status->command = NONE;
						break;
						case DEFAULT_HEAD:			
						status->command = NONE;
						break;					
						case SUPPLY_HEAD:
						status->command = GETSUPPLY;
						break;
						case HEALTH_HEAD:
						status->command = PATROL;					
						break;
						case BUFF_HEAD:
						status->command = GETSUPPLY;
						if(status->health <= status->healthCutOff)
							status->command = PATROL;						
						break;
						case ENEMY_HEAD:			
						status->command = NONE;
						break;
						case BONUS_HEAD:
						status->command = NONE;
						break;					
					}					
				}
			}
			
			return (status->lastSeen_inRange);
		}
};

class CheckHealthStatus : public Node {  // Each task will be a class (derived from Node of course).
	private:
		RobotStatus* status;
		bool boolInv;
	public:
		CheckHealthStatus (RobotStatus* status, bool inv) : status(status) , boolInv(inv) {}
		virtual bool run() override {
			std::cout << "NODE: CHECK_HEALTH_STATUS" << std::endl;

			if (status->health > status->healthCutOff){
				std::cout << "Health is sufficient." << std::endl;  // will return true
				if(!boolInv){
					//status->command = CHASE;
					switch(currentTreeType){
						case NONE:
						status->command = NONE;
						break;
						case DEFAULT_HEAD:			
						status->command = CHASE;
						if(!status->lastSeen_inRange)
							status->command = SEARCH;						
						break;					
						case SUPPLY_HEAD:
						status->command = GETSUPPLY;
						break;
						case HEALTH_HEAD:
						status->command = NONE;					
						break;
						case BUFF_HEAD:
						status->command = NONE;
						break;
						case ENEMY_HEAD:			
						status->command = GETSUPPLY;
						break;
						case BONUS_HEAD:
						status->command = NONE;
						break;					
					}					
				}
			}
			else{
				std::cout << "Health is insufficient." << std::endl;  // will return false
				if(boolInv){
					//status->command = ESCAPE;				
					switch(currentTreeType){
						case NONE:
						status->command = NONE;
						break;
						case DEFAULT_HEAD:			
						status->command = ESCAPE;
						if(!status->lastSeen_inRange)
							status->command = PATROL;
						break;					
						case SUPPLY_HEAD:
						status->command = ESCAPE;
						break;
						case HEALTH_HEAD:
						status->command = NONE;					
						break;
						case BUFF_HEAD:
						status->command = NONE;
						break;
						case ENEMY_HEAD:			
						status->command = GETSUPPLY;
						break;
						case BONUS_HEAD:
						status->command = NONE;
						break;					
					}					
				}
			}
			return (status->health > status->healthCutOff);
		}
};


 
/**************** BLACKBOARD UPDATE OF STATUS VALUES*****************/



int updateStatus(RobotStatus* status, roborts_decision::Blackboard *blackboard){
	
	//VARS TO BE UPDATED BY BLACKBOARD / REFEREE SYSTEM
	
	//double supply; 			//Percent Supply Left
	//double health; 			//Percent Health Left
	//bool buffed;   			//Buff status
	//bool lastSeen_inRange;	//Enemy in range.

	
 
	//EXAMPLE BELOW
	//status-> lastSeen_inRange = blackboard_ -> enemydetected)(?????)
	
	currentTreeType = NONE; 
	status->health = double(blackboard->getHealth());

	//double((blackboard->getMaxHealth() - blackboard->getHealth())/(blackboard->getMaxHealth())) * 100;
	std::cout << "Percent Health: " << status->health << "\t Max Health: " << blackboard->getMaxHealth() << std::endl;	
	status->supply = status->supply - 1;
	status->buffed = false;
	status->lastSeen_inRange = false;
	
	if(status->supply <= status->supplyCutOff){
		currentTreeType = SUPPLY_HEAD;		
	}
	else if(status->health <= status->healthCutOff){
		currentTreeType = HEALTH_HEAD;		
	}
	else if(!status->buffed && status->can_getBuff){
		currentTreeType = BUFF_HEAD;		
	}	
	else if(!status->lastSeen_inRange && status->supply <= supplyCutOff2){
		currentTreeType = ENEMY_HEAD;		
	}
	else{
		currentTreeType = DEFAULT_HEAD;		
	}
}


/********************************************************************/

void runRoot(Selector* root, bool start, roborts_decision::Blackboard *blackboard){
	if(!start){
		while (!root->run()){  // If the operation starting from the root fails, keep trying until it succeeds.
		
			//UPDATE ROBOTSTATUS THROUGH BLACKBOARD
			//RUN DESISION TREE --updates command 
			//run(robotStatus->command)
			
			robotStatus->command = NONE;
			updateStatus(robotStatus, blackboard); //NEED TO UPDATES
			
			//CONTINUE
			
			std::cout << "--------------------" << std::endl;
		}
	}	
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       	 goal_behavior(chassis_executor, blackboard);
  roborts_decision::SupplyBehavior	 supply_behavior(chassis_executor, blackboard, full_path);

	//auto command_thread= std::thread(Command);
	ros::Rate rate(10);
	
	RobotStatus* robotStatus = new RobotStatus {100, 100, false, false, NONE};  

	CheckSupplyStatus* checkSupply = new CheckSupplyStatus (robotStatus, false);
	CheckSupplyStatus* checkSupplyInv = new CheckSupplyStatus (robotStatus, true);

	CheckBuffStatus* checkBuff = new CheckBuffStatus (robotStatus, false);
	CheckBuffStatus* checkBuffInv = new CheckBuffStatus (robotStatus, true);

	CheckLastSeen* checkEnemySeen = new CheckLastSeen (robotStatus, false);
	CheckLastSeen* checkEnemySeenInv = new CheckLastSeen (robotStatus, true);

	CheckHealthStatus* checkHealth = new CheckHealthStatus (robotStatus, false);
	CheckHealthStatus* checkHealthInv = new CheckHealthStatus (robotStatus, true);

	// Original Tree
	/*	
	root->addChild (seq1);
	root->addChild (seq2);
	root->addChild (seq3);
	root->addChild (seq4);

	seq1->addChild(checkSupply);
	seq1->addChild(checkEnemySeen);
	seq1->addChild(checkHealth);

	seq2->addChild(checkSupply);
	seq2->addChild(checkEnemySeen);
	seq2->addChild(inv1);
	inv1->addChild(checkHealthInv);

	seq3->addChild(checkSupply);
	seq3->addChild(inv2);
	inv2->addChild(checkEnemySeenInv);

	seq4->addChild(inv3);
	inv3->addChild(checkSupplyInv);
	*/
	
	// Tree Type: DEFAULT
	Selector* root_1 = new Selector; // Note that root can be either a Sequence or a Selector, since it has only one child.
	Sequence* seq1_1 = new Sequence;  
	Sequence* seq2_1 = new Sequence;  
	Sequence* seq3_1 = new Sequence;  
	Sequence* seq4_1 = new Sequence;  
	
	Inverter* inv1_1 = new Inverter;
	Inverter* inv2_1 = new Inverter;
	Inverter* inv3_1 = new Inverter;
	Inverter* inv4_1 = new Inverter;
	Inverter* inv5_1 = new Inverter;
	
	root_1->addChild (seq1_1);
	root_1->addChild (seq2_1);
	root_1->addChild (seq3_1);
	root_1->addChild (seq4_1);

	seq1_1->addChild(checkSupply);
	seq1_1->addChild(checkEnemySeen);
	seq1_1->addChild(checkHealth);

	seq2_1->addChild(checkSupply);
	seq2_1->addChild(checkEnemySeen);
	seq2_1->addChild(inv1_1);
	inv1_1->addChild(checkHealthInv);

	seq3_1->addChild(checkSupply);
	seq3_1->addChild(inv2_1);
	inv2_1->addChild(checkEnemySeenInv);
	seq3_1->addChild(checkHealth)
	
	seq4_1->addChild(checkSupply);
	seq4_1->addChild(inv4_1);
	inv4_1->addChild(checkEnemySeenInv);
	seq4_1->addChild(inv5_1)
	inv5_1->addChild(checkHealthInv);
	
	
	// Tree Type: SUPPLY_HEAD
	Selector* root_2 = new Selector; // Note that root can be either a Sequence or a Selector, since it has only one child.
	Sequence* seq1_2 = new Sequence;  
	Sequence* seq2_2 = new Sequence;  
	Sequence* seq3_2 = new Sequence;  
	Sequence* seq4_2 = new Sequence;  
	
	Inverter* inv1_2 = new Inverter;
	Inverter* inv2_2 = new Inverter;
	Inverter* inv3_2 = new Inverter;
	Inverter* inv4_2 = new Inverter;

	root_2->addChild (seq1_2);
	root_2->addChild (seq2_2);
	root_2->addChild (seq3_2);

	seq1_2->addChild(checkEnemySeen);
	seq1_2->addChild(checkHealth);
	
	seq2_2->addChild(checkEnemySeen);
	seq2_2->addChild(inv1_2);
	inv1_2->addChild(checkHealthInv);
	
	seq3_1->addChild(inv2_2)
	inv2_2->addChild(checkEnemySeenInv);
	
	
	// Tree Type: HEALTH_HEAD
	Selector* root_3 = new Selector; // Note that root can be either a Sequence or a Selector, since it has only one child.
	Sequence* seq1_3 = new Sequence;  
	Sequence* seq2_3 = new Sequence;  
	
	Inverter* inv1_3 = new Inverter;


	root_3->addChild (seq1_3);
	root_3->addChild (seq2_3);

	seq1_3->addChild(checkEnemySeen);
	
	seq2_3->addChild(inv1_3);
	inv1_3->addChild(checkEnemySeenInv);

	// Tree Type: BUFF_HEAD
	Selector* root_4 = new Selector; // Note that root can be either a Sequence or a Selector, since it has only one child.
	Sequence* seq1_4 = new Sequence;  
	Sequence* seq2_4 = new Sequence;  
	Sequence* seq3_4 = new Sequence;  
	Sequence* seq4_4 = new Sequence;  
	Sequence* seq5_4 = new Sequence;  
	
	Inverter* inv1_4 = new Inverter;
	Inverter* inv2_4 = new Inverter;
	Inverter* inv3_4 = new Inverter;
	Inverter* inv4_4 = new Inverter;
	Inverter* inv5_4 = new Inverter;
	Inverter* inv6_4 = new Inverter;
	
	root_4->addChild (seq1_4);
	root_4->addChild (seq2_4);
	root_4->addChild (seq3_4);
	root_4->addChild (seq4_4);

	seq1_4->addChild(checkHealth);
	seq1_4->addChild(checkSupply);
	
	seq2_4->addChild(checkHealth);
	seq2_4->addChild(inv1_4);
	inv1_4->addChild(checkSupplyInv);
	seq2_4->addChild(checkEnemySeen);

	seq3_4->addChild(checkHealth);
	seq3_4->addChild(inv2_4);
	inv2_4->addChild(checkSupplyInv);
	seq3_4->addChild(inv3_4);
	inv3_4->addChild(checkEnemySeenInv);
	
	seq4_4->addChild(inv4_4);
	inv4_4->addChild(checkHealthInv);
	seq4_4->addChild(checkEnemySeen);

	seq5_4->addChild(inv5_4);
	inv5_4->addChild(checkHealthInv);
	seq5_4->addChild(checkEnemySeen);
	seq5_4->addChild(inv6_4);
	inv6_4->addChild(checkEnemySeenInv);

	
	// Tree Type: ENEMY_HEAD
	Selector* root_5 = new Selector; // Note that root can be either a Sequence or a Selector, since it has only one child.
	Sequence* seq1_5 = new Sequence;  
	Sequence* seq2_5 = new Sequence;  
	Sequence* seq3_5 = new Sequence;  
	Sequence* seq4_5 = new Sequence;  
	
	Inverter* inv1_5 = new Inverter;
	Inverter* inv2_5 = new Inverter;
	Inverter* inv3_5 = new Inverter;
	Inverter* inv4_5 = new Inverter;
	
	root_5->addChild (seq1_5);
	root_5->addChild (seq2_5);
	root_5->addChild (seq3_5);
	root_5->addChild (seq4_5);

	seq1_5->addChild(checkHealth);
	seq1_5->addChild(checkSupply);
	
	seq2_5->addChild(checkHealth);
	seq2_5->addChild(inv1_5);
	inv1_5->addChild(checkSupplyInv);

	seq3_5->addChild(inv2_5);
	inv2_5->addChild(checkHealthInv);
	seq3_5->addChild(checkSupply);
	
	seq4_5->addChild(inv3_5);
	inv3_5->addChild(checkHealthInv);
	seq4_5->addChild(inv4_5);
	inv4_5->addChild(checkSupplyInv);



	bool start = true;
	robotStatus->command = NONE;

	while(ros::ok()){
	    ros::spinOnce();
		updateStatus(robotStatus, blackboard); //NEED TO UPDATE
		
		switch(currentTreeType){
			case NONE:
			std::cout << "NONE Type Tree" << std::endl;
			break;
			case DEFAULT_HEAD:
			std::cout << "DEFAULT_HEAD Type Tree" << std::endl;
			runRoot(root_1, start, blackboard)			
			break;
			
			case SUPPLY_HEAD:
			std::cout << "SUPPLY_HEAD Type Tree" << std::endl;
			runRoot(root_2, start, blackboard)
			break;

			case HEALTH_HEAD:
			std::cout << "HEALTH_HEAD Type Tree" << std::endl;
			runRoot(root_3, start, blackboard)			
			break;
			case BUFF_HEAD:
			std::cout << "BUFF_HEAD Type Tree" << std::endl;
			runRoot(root_4, start, blackboard)			
			break;
			case ENEMY_HEAD:
			std::cout << "ENEMY_HEAD Type Tree" << std::endl;
			runRoot(root_5, start, blackboard)			
			break;
			case BONUS_HEAD:
			std::cout << "BONUS_HEAD Type Tree" << std::endl;	
			break;					
		}
		
		start = false;
		switch (robotStatus->command) {
		  //back to boot area
		  case NONE:
			std::cout << "BOOT BEHAVIOR" << std::endl;
			back_boot_area_behavior.Run();
			break;
			//patrol
		  case PATROL:
			std::cout << "PATROL BEHAVIOR" << std::endl;
			patrol_behavior.Run();	
			break;
			//chase.
		  case CHASE:
			std::cout << "CHASE BEHAVIOR" << std::endl;			
			chase_behavior.Run();
			break;
		  case GETSUPPLY:
			std::cout << "GET SUPPLY BEHAVIOR" << std::endl;			
			supply_behavior.Run();
			break;
			
			//search
		  case SEARCH:
			std::cout << "SEARCH BEHAVIOR" << std::endl;
			search_behavior.Run();
			break;
			//escape.
		  case ESCAPE:
			std::cout << "ESCAPE BEHAVIOR" << std::endl;
			escape_behavior.Run();
			break;
			//goal.
		  case GOAL:
			std::cout << "GOAL BEHAVIOR" << std::endl;
			goal_behavior.Run();
			break;
		  case EXIT:
			std::cout << "EXITING" << std::endl;
			//if (command_thread.joinable()){
			//  command_thread.join();
			//}
			return 0;
		  default:
			break;
		}
		rate.sleep();		
		std::cout << std::endl << "Operation complete.  Behaviour tree exited." << std::endl;
		//std::cin.get();
	}
  
  
  
  
  

  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}

//PREVIOUS VERSION OF TEST... CAN REVERT

/*
  while(ros::ok()){
    ros::spinOnce();
    switch (command) {
      //back to boot area
      case '1':
        back_boot_area_behavior.Run();
        break;
        //patrol
      case '2':
        patrol_behavior.Run();
        break;
        //chase.
      case '3':
        chase_behavior.Run();
        break;
        //search
      case '4':
        search_behavior.Run();
        break;
        //escape.
      case '5':
        escape_behavior.Run();
        break;
        //goal.
      case '6':
        goal_behavior.Run();
        break;
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
        break;
    }
    rate.sleep();
  }
*/