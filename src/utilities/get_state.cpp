#include <thread>
#include <atomic>

#include "ros/ros.h"

#include <iostream>
#include <fstream>

#include <kortex_driver/BaseCyclic_Feedback.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

const double JOINT_1_LIMIT = 2.41;
const double JOINT_3_LIMIT = 2.66;
const double JOINT_5_LIMIT = 2.23;
static const int DOF = 7;

Eigen::VectorXd adjustJointPositions(
  const Eigen::VectorXd& original_positions
) 
{
    Eigen::VectorXd adjusted_positions = original_positions;

    for (int i = 0; i < DOF; i++) {
        if (i == 1 || i == 3 || i == 5) {
            double joint_position = adjusted_positions(i);

            if ((i == 1 && std::abs(joint_position) >= JOINT_1_LIMIT) || 
                (i == 3 && std::abs(joint_position) >= JOINT_3_LIMIT) ||
                (i == 5 && std::abs(joint_position) >= JOINT_5_LIMIT)) {
                
                if (joint_position < 0) {
                    joint_position += 2 * M_PI;
                } else {
                    joint_position -= 2 * M_PI;
                }

                adjusted_positions(i) = joint_position;
            }
        }
    }
    // for (size_t i = 0; i < DOF; i++)
    // {
    //   // std::cout << adjusted_positions(i) << std::endl;
    // }
    

    return adjusted_positions;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_current_state_node");

  ros::NodeHandle n;

  ros::Rate loop_hz(100);

  std::vector<double> positionData(7);
  Eigen::VectorXd current_position(DOF);
  std::vector<double> cartesianData(6);
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/base_feedback");

  current_position.setZero();
  for (size_t i = 0; i < DOF; i++)
  {
    current_position(i) = feedback->actuators[i].position/57.3;
  }
  current_position = adjustJointPositions(current_position);

  std::cout << "Adjusted position (rad): " << std::endl;
  for (int i = 0; i < 7; i++)
  {
    if (i != 6)
    {
      std::cout << current_position(i) << ", ";
    }
    else
    {
      std::cout << current_position(i) << std::endl;
    }
  }

  std::cout << "Current position (rad): " << std::endl;
  for (int i = 0; i < 7; i++)
  {
    positionData[i] = feedback->actuators[i].position;
    if (i != 6)
    {
      std::cout << positionData[i]/57.3 << ", ";
    }
    else
    {
      std::cout << positionData[i]/57.3 << std::endl;
    }
  }

  std::cout << "Current position (deg): " << std::endl;
  for (int i = 0; i < 7; i++)
  {
    positionData[i] = feedback->actuators[i].position;
    if (i != 6)
    {
      std::cout << positionData[i] << ", ";
    }
    else
    {
      std::cout << positionData[i] << std::endl;
    }
  }

  cartesianData[0] = feedback->base.tool_pose_x;
  cartesianData[1] = feedback->base.tool_pose_y;
  cartesianData[2] = feedback->base.tool_pose_z;
  cartesianData[3] = feedback->base.tool_pose_theta_x;
  cartesianData[4] = feedback->base.tool_pose_theta_y;
  cartesianData[5] = feedback->base.tool_pose_theta_z;
  
  std::cout << "Current Cartesian pose of tool frame (ZYX): " << std::endl;
  for (int i = 0; i < cartesianData.size(); i++)
  {
    if (i != cartesianData.size() - 1)
    {
      std::cout << cartesianData[i] << ", ";
    }
    else
    {
      std::cout << cartesianData[i] << std::endl;
    }
  }

  tf2::Quaternion current_quaternion;
  current_quaternion.setRPY(cartesianData[3]/57.3, cartesianData[4]/57.3, cartesianData[5]/57.3);
  std::cout << "Current Cartesian pose of tool frame (Quaternion): " << std::endl;
  for (int i = 0; i < cartesianData.size() - 3; i++)
  {
      std::cout << cartesianData[i] << ", ";
    }

  std::cout << current_quaternion.getX() << ", " << current_quaternion.getY() << ", " << current_quaternion.getZ() << ", " << current_quaternion.getW() << std::endl;

  // test data: 0.3872724, -0.4914169, -0.604657, 0.4928685

  return 0;
}