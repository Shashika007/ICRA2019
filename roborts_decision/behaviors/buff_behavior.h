#ifndef ROBORTS_DECISION_BUFF_BEHAVIOR_H
#define ROBORTS_DECISION_BUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class BuffBehavior {
 public:
  BuffBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    buff_position_.header.frame_id = "map";
    buff_position_.pose.orientation.x = 0;
    buff_position_.pose.orientation.y = 0;
    buff_position_.pose.orientation.z = 0;
    buff_position_.pose.orientation.w = 1;

    buff_position_.pose.position.x = 0;
    buff_position_.pose.position.y = 0;
    buff_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = buff_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = buff_position_.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(buff_position_.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(buff_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 /*|| d_yaw > 0.5*/) {
        chassis_executor_->Execute(buff_position_);

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

    buff_position_.header.frame_id = "map";

    buff_position_.pose.position.x = decision_config.buff_point().x();
    buff_position_.pose.position.z = decision_config.buff_point().z();
    buff_position_.pose.position.y = decision_config.buff_point().y();

    auto buff_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.buff_point().roll(),
                                                                     decision_config.buff_point().pitch(),
                                                                     decision_config.buff_point().yaw());
    buff_position_.pose.orientation = buff_quaternion;

    return true;
  }

  ~BuffBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! supply position
  geometry_msgs::PoseStamped buff_position_;
};
}


#endif //ROBORTS_DECISION_BUFF_BEHAVIOR_H
