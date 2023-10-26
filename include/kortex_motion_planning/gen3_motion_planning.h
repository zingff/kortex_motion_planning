#ifndef GEN3_MOTION_PlANNING_H
#define GEN3_MOTION_PlANNING_H

#include <string>

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
static const std::string TASK_COMPOSER_PLUGIN_SUB_PATH = "/tesseract/task_composer_plugins.yaml";
static const std::string URDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.urdf";
static const std::string SRDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.srdf";

#endif // GEN3_MOTION_PlANNING_H
