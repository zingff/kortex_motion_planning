#include "ros/ros.h"
#include <thread>
#include <atomic>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/ValidateWaypointList.h>
#include <kortex_driver/GetProductConfiguration.h>
#include <kortex_driver/ModelId.h>
#include <kortex_driver/SendTwistCommand.h>
#include <kortex_driver/SendJointSpeedsCommand.h>
#include <kortex_driver/Base_Stop.h>
#include <kortex_driver/StopAction.h>
#include <kortex_driver/PauseAction.h>
#include <kortex_driver/ResumeAction.h>
#include <kortex_driver/SetTwistAngularSoftLimit.h>
#include <kortex_driver/GetTwistHardLimitation.h>
#include <kortex_driver/GetTwistSoftLimitation.h>
#include <kortex_driver/GetTorqueOffset.h>
#include <kortex_driver/SetTorqueOffset.h>
#include <algorithm>
#include <iterator>
#include <std_msgs/Float64.h>
#include <cmath>
#include <kortex_motion_planning/UprightSkewerAction.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

const static std::vector<double> ZERO_TWIST = [] {
    std::vector<double> temp;
    std::generate_n(std::back_inserter(temp), 6, []() { return 0.0; });
    return temp;
}();
const static double WRENCH_FORCE_Z_THRESHOLD = 3.0;

bool twistCommand(
  ros::NodeHandle nh, 
  std::vector<double> twist, 
  int reference_frame)
{
  ros::ServiceClient twist_command_client = nh.serviceClient<kortex_driver::SendTwistCommand>("/base/send_twist_command");
  kortex_driver::SendTwistCommand twist_command;
  twist_command.request.input.reference_frame = reference_frame; // base frame
  twist_command.request.input.duration = 0;

  twist_command.request.input.twist.linear_x = float(twist[0]);
  twist_command.request.input.twist.linear_y = float(twist[1]);
  twist_command.request.input.twist.linear_z = float(twist[2]);
  twist_command.request.input.twist.angular_x = float(twist[3]);
  twist_command.request.input.twist.angular_y = float(twist[4]);
  twist_command.request.input.twist.angular_z = float(twist[5]);

  if (twist_command_client.call(twist_command))
  {
    ROS_INFO("Sending twist command...");
  }
  else
  {
    std::string error_string = "Failed to call twist command service!";
    ROS_ERROR("%s", error_string.c_str());
  }
  return true;
}


bool skewerWithWrenchSensing(
  ros::NodeHandle nh,
  std::vector<double> twist,
  double coefficient,
  double wrench_force_z_threshold
)
{
  double current_wrench_force_z = 0;
  double previous_wrench_force_z = 0;
  double delta_wrench_force_z = 0;
  bool success = false;

  ros::Publisher twist_force_z_publisher = nh.advertise<std_msgs::Float64>("/wrench_force_z", 10);
  ROS_INFO("Begin skewering with wrench sensing...");
  twistCommand(nh, twist, 3);
  
  int i = 0;
  while (true)
  {
    auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/base_feedback");
    current_wrench_force_z = coefficient*feedback->base.tool_external_wrench_force_z + (1 - coefficient) * previous_wrench_force_z;
    delta_wrench_force_z = std::abs(current_wrench_force_z - previous_wrench_force_z);
    previous_wrench_force_z = current_wrench_force_z;
    i++;
    std::cout << "Wrench force Z: " << current_wrench_force_z << std::endl;
    std_msgs::Float64 msg;
    msg.data = current_wrench_force_z;
    twist_force_z_publisher.publish(msg);

    if ((current_wrench_force_z >= wrench_force_z_threshold) && (i>2))
    {
      ROS_INFO("Reached the threshold of the skewer force!");
      twistCommand(nh, ZERO_TWIST, 3);
      break;
    }
    ros::spinOnce();

  }
}

bool wrenchPublish(
  ros::NodeHandle nh,
  double coefficient,
  double wrench_force_z_threshold
)
{
  ros::Publisher twist_force_z_publisher = nh.advertise<std_msgs::Float64>("/wrench_force_z", 10);

  double current_wrench_force_z = 0;
  double previous_wrench_force_z = 0;
  double delta_wrench_force_z = 0;
  bool success = false;

  while (ros::ok())
  {
    auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/base_feedback");
    if (feedback != nullptr) {
      current_wrench_force_z = coefficient * feedback->base.tool_external_wrench_force_z + (1 - coefficient) * previous_wrench_force_z;
      delta_wrench_force_z = std::abs(current_wrench_force_z - previous_wrench_force_z);
      previous_wrench_force_z = current_wrench_force_z;

      std_msgs::Float64 msg;
      msg.data = current_wrench_force_z;
      twist_force_z_publisher.publish(msg);
      success = true;
    }
    
    ros::spinOnce();
  }

  return success;
}

bool uprightSkewerFoodItem(
  kortex_motion_planning::UprightSkewerActionRequest &usfiRequest,
  kortex_motion_planning::UprightSkewerActionResponse & usfiResponse
)
{
  if (usfiRequest.skewer_action_flag = true)
  {
    /* code */
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wrench_sensing_node");
  ros::NodeHandle nh;
  std::vector<double> skewer_twist;

  if (!nh.getParam("/skewer_twist", skewer_twist)) 
  {
  ROS_ERROR("Couldn't retrieve param: skewer_twist.");
  return -1;
  }
  // skewer_twist[2] = 0.5;
  // skewer_twist[2] = 0.02;
  skewerWithWrenchSensing(nh, skewer_twist, 0.2, 8);
  // wrenchPublish(nh, 0.2, WRENCH_FORCE_Z_THRESHOLD);
}
