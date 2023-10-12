#ifndef PLANNING_SERVER_H
#define PLANNING_SERVER_H

#include <kortex_motion_planning/planner_profiles.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <tesseract_common/timer.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/profiles/min_length_profile.h>
#include <tesseract_task_composer/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/task_composer_problem.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_plugin_factory.h>
#include <tesseract_support/tesseract_support_resource_locator.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit/robot_model/robot_model.h>
#include <kortex_motion_planning/GenerateKortexMotionPlan.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "GEN3";
static const std::string PROFILE2 = "GEN3_FREESPACE";
static const std::string PLANNING_SERVICE = "/my_gen3/motion_planning_server";
static const std::string MONITOR_NAMESPACE = "gen3_environment";
static const std::string TASK_PIPELINE = "Gen3Pipeline";
static const std::string STATE_FEEDBACK_TOPIC = "/my_gen3/base_feedback";
static const double MAX_TCP_SPEED = 0.2;
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
static const int DOF = 7;
static const std::string MANIPULATOR_GROUP_NAME = "manipulator";
static const std::string WORKING_FRAME_NAME = "base_link";
static const std::string TCP_FRAME_NAME = "tool_frame";
static const std::string TASK_COMPOSER_PLUGIN_PKG_NAME = "kortex_motion_planning";
static const std::string TASK_COMPOSER_PLUGIN_SUB_PATH = "/config/tesseract/task_composer_plugins.yaml";
static const std::string URDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.urdf";
static const std::string SRDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.srdf";

class PlanningServer
{
public:
  PlanningServer(ros::NodeHandle n);
  bool plan(
      kortex_motion_planning::GenerateKortexMotionPlan::Request &req,
      kortex_motion_planning::GenerateKortexMotionPlan::Response &res);

private:
  ros::NodeHandle nh_;
  bool plotting_ = true;
  bool rviz_ = true;
  bool ifopt_ = false;
  bool verbose_ = false;
  bool debug_ = false;
  int method_id_;

  tesseract_common::JointTrajectory joint_trajectory_;

  std::string urdf_xml_string_, srdf_xml_string_;
  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  moveit::core::RobotModelPtr robot_model_;

  tesseract_environment::Environment::Ptr env_;
  tesseract_planning::ProfileDictionary::Ptr profile_dictionary_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_visualization::Visualization::Ptr plotter_;

  Eigen::VectorXd current_joint_position_;
  std::vector<std::string> joint_names_;
  tesseract_common::ManipulatorInfo manipulator_info_;

  void getCurrentPosition();
  tesseract_common::JointTrajectory tcpSpeedLimiter(
      const tesseract_common::JointTrajectory &input_trajectory,
      const double max_speed,
      const std::string tcp);
  tesseract_planning::CompositeInstruction createProgram(
      const tesseract_common::ManipulatorInfo &info,
      const Eigen::Isometry3d &end_pose);

};

#endif // PLANNING_SERVER_H