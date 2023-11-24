#include <QApplication>
#include "qcustomplot.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/StopAction.h>

static const int DOF = 7;
static const std::string BASE_FEEDBACK_TOPIC = "/base_feedback";
QCustomPlot* customPlot;

void updatePlot(const Eigen::VectorXd& tau_measured, qint64 startingTime) {
    qint64 currentTime = (QDateTime::currentDateTime().toMSecsSinceEpoch() - startingTime);

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
    customPlot->yAxis->setRange(-100, 100);

    customPlot->replot();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_torque_plot");
    ros::NodeHandle nh;

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

    customPlot->xAxis->setLabel("Time (ms)");
    customPlot->yAxis->setLabel("Torque (Nm)");

    customPlot->legend->setVisible(true);
    customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignLeft);

    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    customPlot->replot();
    customPlot->show();

    Eigen::VectorXd tau_measured(DOF);

    QTimer dataTimer;
    dataTimer.start(100);  

    qint64 currentTimeStamp = QDateTime::currentDateTime().toMSecsSinceEpoch();
    QObject::connect(&dataTimer, &QTimer::timeout, [&]() {
        auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(BASE_FEEDBACK_TOPIC);
        for (size_t i = 0; i < DOF; i++) {
            tau_measured(i) = -feedback->actuators[i].torque;
        }
        updatePlot(tau_measured, currentTimeStamp);
    });

    return app.exec();
}

