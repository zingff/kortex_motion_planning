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
#include <kortex_motion_planning/UprightSkewerAction.h>
#include <kortex_motion_planning/ros_utilities.hpp>
#include <kortex_driver/SendTwistCommand.h>
#include <std_msgs/Float64.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

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
  ros::Publisher twist_force_z_publisher_;
  ros::ServiceClient gripper_cmd_client_;
  ros::ServiceClient twist_command_client_;


  SimpleMpe(ros::NodeHandle nh_, std::string planning_group_);


  bool twistCommand(
    std::vector<double> twist, 
    int reference_frame)
  {
    kortex_driver::SendTwistCommand twist_command;
    twist_command.request.input.reference_frame = reference_frame; // base frame
    twist_command.request.input.duration = 0;

    twist_command.request.input.twist.linear_x = float(twist[0]);
    twist_command.request.input.twist.linear_y = float(twist[1]);
    twist_command.request.input.twist.linear_z = float(twist[2]);
    twist_command.request.input.twist.angular_x = float(twist[3]);
    twist_command.request.input.twist.angular_y = float(twist[4]);
    twist_command.request.input.twist.angular_z = float(twist[5]);

    if (twist_command_client_.call(twist_command))
    {
      ROS_INFO(GREEN "Sending twist command..." RESET);
    }
    else
    {
      ROS_INFO(RED "Failed to call twist command service!." RESET);
    }
    return true;
  }

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

  bool getUtensilAction();

  bool uprightSkewerAction(
    kortex_motion_planning::UprightSkewerActionRequest &usaRequest,
    kortex_motion_planning::UprightSkewerActionResponse &usaResponse
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

  // Upright skewer action
  std::vector<double> upright_skewer_twist_;
  double lpf_coefficient_;
  double wrench_force_z_threshold_;
  std::vector<double> zero_twist_;

  std::string base_cyclic_feedback_topic_;
  std::string send_gripper_command_service_;
  std::string send_twist_command_service_;

  int reference_frame_;

  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  double planning_time_;
  double planning_attempts_number_;

  // get utensil
  std::vector<double> holder_positions_;
  std::vector<double> utensil_positions;
};

#endif  // SIMPLE_MPE_HPP