#include <ros/ros.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>

static const std::string GRIPPER_COMMAND_SERVICE_NAME = "/my_gen3/base/send_gripper_command";

bool gripperCommand(ros::NodeHandle n, double value)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>(GRIPPER_COMMAND_SERVICE_NAME);
  kortex_driver::SendGripperCommand service_send_gripper_command;

  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  service_send_gripper_command.request.input.gripper.finger.push_back(finger);
  service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

  if (service_client_send_gripper_command.call(service_send_gripper_command))  
  {
    ROS_INFO("Gripper command: %f.", value);
  }
  else
  {
    std::string error_string = "Failed to call SendGripperCommand";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gen3_gripper_command");
  ros::NodeHandle n;
  while (true)
  {
    double cmd;
    std::cout << "Please input the gripper command" << std::endl;
    std::cin >> cmd;
    if (cmd <= 1.0){
      gripperCommand(n, cmd);
    }
    else if (cmd == 9)
    {
      exit(1);
    }
    else
    {
      std::cout << "Invalid input" << std::endl;
    }
  }
}