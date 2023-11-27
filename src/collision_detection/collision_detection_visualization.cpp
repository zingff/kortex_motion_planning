#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <QApplication>
#include "qcustomplot.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/StopAction.h>
#include <iostream>
#include <fstream> 
#include <string>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

#ifndef KORTEX_CONFIG_DIR
  #define KORTEX_CONFIG_DIR "/home/zing/mealAssistiveRobot/sla_ws/src/kortex_motion_planning/config"
#endif

static const int DOF = 7;
static const std::string BASE_FEEDBACK_TOPIC = "/base_feedback";
QCustomPlot* customPlot;
static const std::vector<double> TAU_THRESHOLD = {3.0, -3.0};

Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& degreesVector) {
    return (M_PI / 180.0) * degreesVector;
}

void updatePlot(const Eigen::VectorXd& tau_measured, qint64 currentTime, QCPItemLine* linePos, QCPItemLine* lineNeg) {
    // std::cout << "Updating plot" << std::endl;
    // qint64 currentTime = (QDateTime::currentDateTime().toMSecsSinceEpoch() - startingTime);

    customPlot->graph(0)->addData(currentTime, tau_measured(0));
    customPlot->graph(1)->addData(currentTime, tau_measured(1));
    customPlot->graph(2)->addData(currentTime, tau_measured(2));
    customPlot->graph(3)->addData(currentTime, tau_measured(3));
    customPlot->graph(4)->addData(currentTime, tau_measured(4));
    customPlot->graph(5)->addData(currentTime, tau_measured(5));
    customPlot->graph(6)->addData(currentTime, tau_measured(6));
    QPen pen1;
    pen1.setColor(QColor(255, 0, 0)); // Red: RGB(255, 0, 0)
    pen1.setWidth(2);
    customPlot->graph(0)->setPen(pen1);
    QPen pen2;
    pen2.setColor(QColor(0, 255, 0)); // Green: RGB(0, 255, 0)
    pen2.setWidth(2);
    customPlot->graph(1)->setPen(pen2);
    QPen pen3;
    pen3.setColor(QColor(0, 0, 255)); // Blue: RGB(0, 0, 255)
    pen3.setWidth(2);
    customPlot->graph(2)->setPen(pen3);
    QPen pen4;
    pen4.setColor(QColor(255, 255, 0)); // Yellow: RGB(255, 255, 0)
    pen4.setWidth(2);
    customPlot->graph(3)->setPen(pen4);
    QPen pen5;
    pen5.setColor(QColor(0, 255, 255)); // Cyan: RGB(0, 255, 255)
    pen5.setWidth(2);
    customPlot->graph(4)->setPen(pen5);
    QPen pen6;
    pen6.setColor(QColor(255, 0, 255)); // Magenta: RGB(255, 0, 255)
    pen6.setWidth(2);
    customPlot->graph(5)->setPen(pen6);
    QPen pen7;
    pen7.setColor(QColor(255, 165, 0)); // Orange: RGB(255, 165, 0)
    pen7.setWidth(2);
    customPlot->graph(6)->setPen(pen7);
    customPlot->xAxis->setRange(currentTime + 1000, currentTime - 59000);
    customPlot->yAxis->setRange(-10, 10);

    linePos->start->setCoords(currentTime - 59000, TAU_THRESHOLD.at(0));
    linePos->end->setCoords(currentTime + 1000, TAU_THRESHOLD.at(0));
    lineNeg->start->setCoords(currentTime - 59000, *(TAU_THRESHOLD.end() - 1));
    lineNeg->end->setCoords(currentTime + 1000, *(TAU_THRESHOLD.end() - 1));

    customPlot->replot();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_detection_visualization");
    ros::NodeHandle nh;

    // Initialize QT plot
    QApplication app(argc, argv);
    customPlot = new QCustomPlot();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->setGeometry(100, 100, 800, 600);
    customPlot->graph(0)->setName("Joint 1");
    customPlot->graph(1)->setName("Joint 2");
    customPlot->graph(2)->setName("Joint 3");
    customPlot->graph(3)->setName("Joint 4");
    customPlot->graph(4)->setName("Joint 5");
    customPlot->graph(5)->setName("Joint 6");
    customPlot->graph(6)->setName("Joint 7");
    QCPItemLine *linePos = new QCPItemLine(customPlot);
    QCPItemLine *lineNeg = new QCPItemLine(customPlot);
    customPlot->xAxis->setLabel("Time (ms)");
    customPlot->yAxis->setLabel("Delta torque (Nm)");
    customPlot->legend->setVisible(true);
    customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignLeft);

    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    customPlot->replot();
    customPlot->show();

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
    Eigen::VectorXd a_measured = Eigen::VectorXd::Zero(DOF); // not available yet
    Eigen::VectorXd tau_measured = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd tau_estimated = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd delta_tau = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd F_threshold = Eigen::VectorXd::Ones(DOF);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

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

    // Main loop for force checking and plotting
    while (ros::ok())
    {
      auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(BASE_FEEDBACK_TOPIC);
      qint64 currentTime = (QDateTime::currentDateTime().toMSecsSinceEpoch() - startingTimeStamp);
      for (size_t i = 0; i < DOF; i++)
      {
        q_measured(i) = feedback->actuators[i].position;
        v_measured(i) = feedback->actuators[i].velocity;
        // a_measured(i) = feedback->actuators[i].
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
      delta_tau = (tau_estimated - tau_measured);
      // std::cout << "Delta torque " << delta_tau.transpose() << std::endl;

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

      QTimer dataTimer;
      dataTimer.start(100);  

      // Update the plot with the measured torque
      // qint64 currentTimeStamp = QDateTime::currentDateTime().toMSecsSinceEpoch();
      updatePlot(delta_tau, currentTime, linePos, lineNeg);

      // Process Qt events to update the plot
      QApplication::processEvents();

      // Check for excessive joint torque
      if (delta_tau.maxCoeff() >= TAU_THRESHOLD.at(0))
      {
        ROS_WARN("Excessive joint torque detected! Stopping!");
        if (stop_client.call(stop_action))
        {
            ROS_INFO("Succeeded to Stop!");
        }
        else
        {
            ROS_ERROR("Failed to stop! Press emergency stop!");
            // return 1;
        }
      }
    }

    return app.exec();
}


