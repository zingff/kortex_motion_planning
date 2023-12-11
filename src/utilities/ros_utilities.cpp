#include <kortex_motion_planning/ros_utilities.hpp>

#define RESET   "\033[0m"
#define RED     "\033[1;38;5;203m"
#define GREEN   "\033[1;38;5;190m"
#define YELLOW   "\033[1;38;5;184m"
#define CYAN   "\033[1;38;5;122m"
#define PURPLE   "\033[1;38;5;183m"

template <typename ParamType>
bool getROSParam(
  ros::NodeHandle nh, 
  const std::string& param_name, 
  ParamType& param_variable) {
  if (!nh.getParam(param_name, param_variable)) {
    ROS_INFO(RED "Couldn't retrieve param: %s." RESET, param_name.c_str());
    return false;
  }
  return true;
}

template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, int& param_variable);
template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, double& param_variable);
template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, std::string& param_variable);
template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, std::vector<double>& param_variable);




