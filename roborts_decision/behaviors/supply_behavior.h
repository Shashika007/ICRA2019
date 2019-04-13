#ifndef ROBORTS_DECISION_SUPPLY_BEHAVIOR_H
#define ROBORTS_DECISION_SUPPLY_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class SupplyBehavior {
 public:
  SupplyBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    supply_position_.header.frame_id = "map";
    supply_position_.pose.orientation.x = 0;
    supply_position_.pose.orientation.y = 0;
    supply_position_.pose.orientation.z = 0;
    supply_position_.pose.orientation.w = 1;

    supply_position_.pose.position.x = 0;
    supply_position_.pose.position.y = 0;
    supply_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = supply_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = supply_position_.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(supply_position_.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(supply_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_->Execute(supply_position_);

      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    supply_position_.header.frame_id = "map";

    supply_position_.pose.position.x = decision_config.supply_point().x();
    supply_position_.pose.position.z = decision_config.supply_point().z();
    supply_position_.pose.position.y = decision_config.supply_point().y();

    auto supply_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.supply_point().roll(),
                                                                     decision_config.supply_point().pitch(),
                                                                     decision_config.supply_point().yaw());
    supply_position_.pose.orientation = supply_quaternion;

    return true;
  }

  ~SupplyBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! supply position
  geometry_msgs::PoseStamped supply_position_;
};
}


#endif //ROBORTS_DECISION_SUPPLY_BEHAVIOR_H
