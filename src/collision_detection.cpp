#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <fstream> 
#include <string>

#include <ros/ros.h>
#include <kortex_driver/StopAction.h>

#include <Eigen/Core>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/StopAction.h>
#include <cmath>

#ifndef KORTEX_CONFIG_DIR
  #define KORTEX_CONFIG_DIR "/home/zing/mealAssistiveRobot/sla_ws/src/kortex_motion_planning/config"
#endif

static const int DOF = 7;
static const std::string BASE_FEEDBACK_TOPIC = "/my_gen3/base_feedback";

Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& degreesVector) {
    return (M_PI / 180.0) * degreesVector;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle nh;

    // Path to urdf model, you can also pass the model path by the second param argv[1]
    const std::string urdf_filename = (argc<=1) ? KORTEX_CONFIG_DIR + std::string("/robot/gen3_robotiq_2f_85.urdf") : argv[1];
    const std::string data_dir = KORTEX_CONFIG_DIR + std::string("/data/");

    // Load the URDF model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    // Build data related to model
    pinocchio::Data data(model);

    Eigen::VectorXd q_f = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd F_f = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd F_e = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd delta_F = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd F_thre = Eigen::VectorXd::Ones(DOF);

    // for (size_t i = 0; i < model.frames.size(); i++)
    // {
    //   std::cout << model.frames[i] << std::endl;
    // }

    ros::ServiceClient stop_client = nh.serviceClient<kortex_driver::StopAction>("/my_gen3/base/stop_action");
    kortex_driver::StopAction stop_action;

    // Force checking loop
    while (true)
    {
      // Joint states feedback from robot base
      auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(BASE_FEEDBACK_TOPIC);
      for (size_t i = 0; i < DOF; i++)
      {
        q_f(i) = feedback->actuators[i].position;
        F_f(i) = - feedback->actuators[i].torque;

      }
      q_f = degreesToRadians(q_f);

      // Convert joint position to pinocchio form 
      q(0) = cos(q_f(0));
      q(1) = sin(q_f(0));

      q(2) = q_f(1); 

      q(3) = cos(q_f(2));
      q(4) = sin(q_f(2));

      q(5) = q_f(3); 

      q(6) = cos(q_f(4));
      q(7) = sin(q_f(4));

      q(8) = q_f(5);
      
      q(9) = cos(q_f(6));
      q(10) = sin(q_f(6));

      pinocchio::rnea(model, data, q, v, a);

      // std::cout << "Joint torque: " << data.tau.head(7).transpose() << std::endl;
      F_e = data.tau.head(DOF);
      delta_F = (F_e - F_f).cwiseAbs();
      std::cout << "Delta torque " << delta_F.transpose() << std::endl;

      std::ofstream outFile(data_dir + "data.txt", std::ios::app);
      if (outFile.is_open())
      {
          outFile << "q_f: " << q_f.transpose() << std::endl;
          outFile << "F_e: " << F_e.transpose() << std::endl;
          outFile << "F_f: " << F_f.transpose() << std::endl;
          outFile << "delta_F: " << delta_F.transpose() << std::endl;
          outFile.close();
      }
      else
      {
          ROS_ERROR("Failed to open the data file for writing!");
          return 1;
      }

      if (delta_F.maxCoeff() >= 2)
      {
        ROS_WARN("Excessive joint torque detected! Stopping!");
        if (stop_client.call(stop_action))
        {
            ROS_INFO("Succeeded to Stop!");
        }
        else
        {
            ROS_ERROR("Failed to stop! Press emergency stop!");
            return 1;
        }
      }
    }

    return 0;
}
