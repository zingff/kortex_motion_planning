#ifndef SIMLE_MPE_HPP
#define SIMLE_MPE_HPP

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <console_bridge/console.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <vector>
#include <string>
#include <kortex_motion_planning/KortexSimpleJmpe.h>
#include <kortex_motion_planning/KortexSimpleCmpe.h>
#include <kortex_motion_planning/SendGripperCommand.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_motion_planning/GetUtensil.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

#define RESET   "\033[0m"
#define RED     "\033[1;38;5;202m"
#define GREEN   "\033[1;38;5;112m"
#define YELLOW   "\033[1;38;5;227m"
#define CYAN   "\033[1;38;5;123m"
#define WHITE   "\033[1;38;5;195m"
#define WHITE2   "\033[1;38;5;195m"

class SimpleMpe
{
private:
  // Move arm functions:
  void setJointGroup(double joint_0, 
                     double joint_1, 
                     double joint_2,
                     double joint_3, 
                     double joint_4, 
                     double joint_5, 
                     double joint_6);

  bool moveToJoint(const std::vector<double>& joint_positions);

  bool moveToCartesian(geometry_msgs::Pose);

public:
  ros::Publisher gripper_command_pub_;
  ros::Publisher display_publisher_;
  ros::ServiceClient gripper_cmd_client_;


  SimpleMpe(ros::NodeHandle nh_, std::string planning_group_);

  bool kortexSimpleJointMotionPlanningAndExecution(
    kortex_motion_planning::KortexSimpleJmpe::Request &ksjmpeRequest,
    kortex_motion_planning::KortexSimpleJmpe::Response &ksjmpeResponse
  );

  bool kortexSimpleCartesianMotionPlanningAndExecution(
    kortex_motion_planning::KortexSimpleCmpe::Request &kscmpeReuqest, 
    kortex_motion_planning::KortexSimpleCmpe::Response & kscmpeResponse
  );

  bool sendKortexGripperCommand(
    kortex_motion_planning::SendGripperCommandRequest &skgcRequest,
    kortex_motion_planning::SendGripperCommandResponse &skgcResponse
  );

  bool getUtensil(
    kortex_motion_planning::GetUtensilRequest &guRequest,
    kortex_motion_planning::GetUtensilResponse &guResponse
  );

  moveit::core::RobotStatePtr current_state_;
  std::vector<double> joint_positions_;
  std::string planning_group_name_;
  PlanningScenePtr planning_scene_ptr_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  MoveGroupPtr move_group_ptr_;
  moveit::planning_interface::MoveGroupInterface::Plan joint_motion_plan_;
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_motion_plan_;
  // todo: move to private
  void getCurrentPositions();

  void moveToFeedingInitialPositions();

  void sendGripperCommand(double gripper_position_);
  void closeGripper();
  void openGripper();
  control_msgs::GripperCommandActionGoal gripper_cmd_;

  // Planning variables
  geometry_msgs::Pose target_pose_;
  geometry_msgs::Vector3 orientation_;
  geometry_msgs::Point pose;
  geometry_msgs::Point position;
  tf2::Quaternion q;
};

#endif  // SIMPLE_MPE_HPP