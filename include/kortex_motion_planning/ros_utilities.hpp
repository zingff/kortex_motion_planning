#ifndef ROS_UTILITIES_HPP
#define ROS_UTILITIES_HPP

#include <ros/ros.h>
#include <string>

template <typename ParamType>
bool getROSParam(ros::NodeHandle nh, const std::string& param_name, ParamType& param_variable);

#endif // ROS_UTILITIES_HPP