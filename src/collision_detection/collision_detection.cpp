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
#include <QApplication>
#include "qcustomplot.h"

#ifndef KORTEX_CONFIG_DIR
  #define KORTEX_CONFIG_DIR "/home/zing/mealAssistiveRobot/sla_ws/src/kortex_motion_planning/config"
#endif

#define RESET   "\033[0m"
#define RED     "\033[1;31m"
#define GREEN   "\033[1;32m"
#define YELLOW   "\033[1;33m"
#define CYAN   "\033[1;38;5;123m"


static const int DOF = 7;
static const std::string BASE_FEEDBACK_TOPIC = "/base_feedback";
static const std::vector<double> TAU_THRESHOLD = {3.4, -3.4};
static const std::string COLLISION_STATUS_MESSAGE_NAME = "/kortex_motion_planning/collision_detection";

Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& degreesVector) {
    return (M_PI / 180.0) * degreesVector;
}

// TODO: merge this code and joint_torque_realtime_plot 
// and update the data structure of joint torque/delta torque 
// with the time stamp used in qt plot (1+7 columns)
// done

int main(int argc, char** argv)
{
    ROS_INFO(GREEN "Collision detecting for Kinova Gen3" RESET);
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle nh;
    ros::Publisher collision_status_pub = nh.advertise<std_msgs::Bool>(COLLISION_STATUS_MESSAGE_NAME, 10);

    QApplication app(argc, argv);

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

    // for (size_t i = 0; i < model.joints.size(); i++)
    // {
    //   std::cout << model.joints[i] << std::endl;
    // }

    ros::ServiceClient stop_client = nh.serviceClient<kortex_driver::StopAction>("/base/stop_action");
    kortex_driver::StopAction stop_action;

    qint64 startingTimeStamp = QDateTime::currentDateTime().toMSecsSinceEpoch();

    // Generate timestamp and create file name with it
    std::time_t now = std::time(nullptr);
    std::tm *ltm = std::localtime(&now);
    std::stringstream ss;
    ss << std::put_time(ltm, "%Y%m%d%H%M%S");
    std::string timestamp = ss.str();
    std::string filename = data_dir + "collisionDetectionData" + timestamp + ".csv";

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
      // std::cout << "Delta torque " << delta_tau.transpose() << std::endl;

      qint64 currentTime = (QDateTime::currentDateTime().toMSecsSinceEpoch() - startingTimeStamp);
      std::ofstream outFile(filename, std::ios::app);  
      if (outFile.is_open())
      {
          outFile << currentTime << ", ";
          for (int i = 0; i < delta_tau.size(); ++i) {
              outFile << delta_tau(i);
              outFile << ", ";
          }
          for (int i = 0; i < tau_estimated.size(); ++i) {
              outFile << tau_estimated(i);
              outFile << ", ";
          }
          for (int i = 0; i < tau_measured.size(); ++i) {
              outFile << tau_measured(i);
              // if (i < tau_measured.size() - 1)
                  outFile << ", ";
          }
          for (int i = 0; i < q_measured.size(); ++i) {
              outFile << q_measured(i);
              // if (i < q_measured.size() - 1)
                  outFile << ", ";
          }
          for (int i = 0; i < v_measured.size(); ++i) {
              outFile << q_measured(i);
              if (i < v_measured.size() - 1)
                  outFile << ", ";
          }
          outFile << std::endl;
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
        // std::cout << "Delta torque " << delta_tau.transpose() << std::endl;
        ROS_INFO(CYAN "Delta tau: " RESET);
        ROS_INFO_STREAM(CYAN << delta_tau.transpose() << RESET);
        ROS_INFO(YELLOW "Excessive joint torque detected! Stopping!" RESET);
        if (stop_client.call(stop_action))
        {
          ROS_INFO(GREEN "Succeeded to Stop!" RESET);
        }
        else
        {
          ROS_INFO(RED "Failed to stop! Press emergency stop!" RESET);
          // return 1;
        }

        collision_state.data = true;
        ros::Time start_time = ros::Time::now();
        while (ros::Time::now() - start_time < ros::Duration(1.0)) {
            collision_status_pub.publish(collision_state);
            ros::spinOnce();
            ros::Duration(0.1).sleep();  
        }

        collision_state.data = false;
        collision_status_pub.publish(collision_state);
      }
      collision_status_pub.publish(collision_state);
      ros::spinOnce();
    }
    return 0;
}
