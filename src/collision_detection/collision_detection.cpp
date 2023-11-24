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
#include <std_msgs/Bool.h>

#ifndef KORTEX_CONFIG_DIR
  #define KORTEX_CONFIG_DIR "/home/zing/mealAssistiveRobot/sla_ws/src/kortex_motion_planning/config"
#endif

static const int DOF = 7;
static const std::string BASE_FEEDBACK_TOPIC = "/base_feedback";
static const std::vector<double> TAU_THRESHOLD = {3.0, -3.0};

Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& degreesVector) {
    return (M_PI / 180.0) * degreesVector;
}

// TODO: merge this code and joint_torque_realtime_plot 
// and update the data structure of joint torque/delta torque 
// with the time stamp used in qt plot (1+7 columns)
// done

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle nh;
    ros::Publisher collision_status_pub = nh.advertise<std_msgs::Bool>("/collision_status", 10);

    // Path to urdf model, you can also pass the model path by the second param argv[1]
    const std::string urdf_filename = (argc<=1) ? KORTEX_CONFIG_DIR + std::string("/robot/gen3_robotiq_2f_85.urdf") : argv[1];
    const std::string data_dir = KORTEX_CONFIG_DIR + std::string("/data/");

    // Load the URDF model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    // Build data related to model
    pinocchio::Data data(model);

    // Initialize dynamic variables
    Eigen::VectorXd q_measured = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd v_measured = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd a_measured = Eigen::VectorXd::Zero(DOF); // acquire via momentum observer
    Eigen::VectorXd tau_measured = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd tau_estimated = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd delta_tau = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd F_threshold = Eigen::VectorXd::Ones(DOF);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    for (size_t i = 0; i < model.joints.size(); i++)
    {
      std::cout << model.joints[i] << std::endl;
    }

    ros::ServiceClient stop_client = nh.serviceClient<kortex_driver::StopAction>("/base/stop_action");
    kortex_driver::StopAction stop_action;

    // Force checking loop
    while (true)
    {
      // Joint states feedback from robot base
      auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(BASE_FEEDBACK_TOPIC);
      for (size_t i = 0; i < DOF; i++)
      {
        q_measured(i) = feedback->actuators[i].position;
        v_measured(i) = feedback->actuators[i].velocity;
        tau_measured(i) = - feedback->actuators[i].torque;
        // std::cout << i << ": " << feedback->actuators[i].current_motor << "; " <<
        // feedback->actuators[i].torque << "; " << feedback->actuators[i].torque/feedback->actuators[i].current_motor << std::endl;
      }
      q_measured = degreesToRadians(q_measured);
      v_measured = degreesToRadians(v_measured);

      // Formulate joint position into Lie algebra form
      q(0) = cos(q_measured(0));
      q(1) = sin(q_measured(0));
      q(2) = q_measured(1); 
      q(3) = cos(q_measured(2));
      q(4) = sin(q_measured(2));
      q(5) = q_measured(3); 
      q(6) = cos(q_measured(4));
      q(7) = sin(q_measured(4));
      q(8) = q_measured(5);
      q(9) = cos(q_measured(6));
      q(10) = sin(q_measured(6));
      for (size_t i = 0; i < DOF; i++)
      {
        v(i) = v_measured(i);
      }
      pinocchio::rnea(model, data, q, v, a);
      // std::cout << "Joint torque: " << data.tau.head(7).transpose() << std::endl;
      tau_estimated = data.tau.head(DOF);
      delta_tau = (tau_estimated - tau_measured).cwiseAbs();
      std::cout << "Delta torque " << delta_tau.transpose() << std::endl;

      std::ofstream outFile(data_dir + "data.txt", std::ios::app);
      if (outFile.is_open())
      {
          outFile << "q_measured: " << q_measured.transpose() << std::endl;
          outFile << "tau_estimated: " << tau_estimated.transpose() << std::endl;
          outFile << "tau_measured: " << tau_measured.transpose() << std::endl;
          outFile << "delta_tau: " << delta_tau.transpose() << std::endl;
          outFile.close();
      }
      else
      {
          ROS_ERROR("Failed to open the data file for writing!");
          return 1;
      }

      // Check for excessive joint torque
      std_msgs::Bool collision_state;
      collision_state.data = false;
      if (delta_tau.maxCoeff() >= TAU_THRESHOLD.at(0))
      {
        ROS_WARN("Excessive joint torque detected! Stopping!");
        if (stop_client.call(stop_action))
        {
          collision_state.data = true;
          ROS_INFO("Succeeded to Stop!");
        }
        else
        {
          collision_state.data = false;
          ROS_ERROR("Failed to stop! Press emergency stop!");
          return 1;
        }
      }
      collision_status_pub.publish(collision_state);
      ros::spinOnce();
    }
    return 0;
}
