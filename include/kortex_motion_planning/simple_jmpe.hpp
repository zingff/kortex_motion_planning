#ifndef SIMLE_JMPE_HPP
#define SIMLE_JMPE_HPP

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <console_bridge/console.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <vector>
#include <string>
#include <kortex_motion_planning/KortexSimpleJmpe.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class SimpleJmpe
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

  void moveToJoint(std::vector<double>);

public:
  ros::Publisher gripper_command_pub_;
  ros::Publisher display_publisher_;

  SimpleJmpe(ros::NodeHandle nh_, std::string planning_group_);

  bool kortexSimpleJointMotionPlanningAndExecution(kortex_motion_planning::KortexSimpleJmpe::Request &ksjmpeRequest,
                                kortex_motion_planning::KortexSimpleJmpe::Response &ksjmpeResponse);


  moveit::core::RobotStatePtr current_state_;
  std::vector<double> joint_positions_;
  std::string planning_group_name_;
  PlanningScenePtr planning_scene_ptr_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  MoveGroupPtr move_group_ptr_;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;
  // todo: move to private
  void getCurrentPositions();

  void goTop();

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

#endif  // SIMPLE_JMPE_HPP