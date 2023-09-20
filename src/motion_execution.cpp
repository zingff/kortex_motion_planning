#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kortex_motion_planning/ExecuteMotionPlan.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

static const std::string MOTION_EXECUTION_SERVICE = "/my_gen3/motion_execution_server";
static const std::string MOTION_EXECUTION_CLIENT = "/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory";
static const std::string MOTION_EXECUTION_SERVER = "/my_gen3/motion_exection_server";
static const std::string STATE_FEEDBACK_TOPIC = "/my_gen3/base_feedback";
static const int DOF = 7;
static const double JOINT_DISTANCE_THRESHOLD = 0.01;
static const double EXECUTION_TIME_THRESHOLD = 15.0;


// using FJT = control_msgs::FollowJointTrajectoryAction;
// TODO: modify the last waypoint to target


class MotionExecutionServer
{
  public:
  MotionExecutionServer(ros::NodeHandle n)
  : nh_(n)
  // , motion_execution_client_(MOTION_EXECUTION_CLIENT, true)
  {
    // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> motion_execution_client_("my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory", false);    
    // motion_execution_server_ = nh_.advertiseService(MOTION_EXECUTION_SERVER, &MotionExecutionServer::executionMotionPlan, this);
    // ROS_INFO("Ready to execute motion for Kinova Gen3!");


  }

  // Note: cb should be bool
  bool executionMotionPlan (kortex_motion_planning::ExecuteMotionPlan::Request &empRequest,
  kortex_motion_planning::ExecuteMotionPlan::Response &empResponse)
  {
    try
    {
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> motion_execution_client_("my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory", false);    
      // Check whether motion execution client is connected
      motion_execution_client_.waitForServer(ros::Duration(1.0));
      if(!motion_execution_client_.isServerConnected())
      {
        ROS_ERROR("Action client is not yet connected!");
      }

      // Send motion plan trajectory
      control_msgs::FollowJointTrajectoryGoal goal_msg;
      goal_msg.trajectory = empRequest.motion_plan;

      // Check if the robot is static
      // TODO: add timer
      trajectory_msgs::JointTrajectoryPoint joint_state_1, joint_state_2;
      joint_state_2 = getJointState();
      ros::Duration(1.0).sleep();
      joint_state_1 = getJointState();
      double joint_distance = 0;
      for (int i = 0; i < DOF; i++)
      {
        double diff = joint_state_1.positions[i] - joint_state_2.positions[i];
        joint_distance += diff * diff;
      }
      joint_distance = std::sqrt(joint_distance);
      // TODO: check for a certain of time
      if (joint_distance >= JOINT_DISTANCE_THRESHOLD)
      {
        ROS_WARN("Please check if the robot is moving");
        ROS_WARN("Keep waiting for some seconds.");
        ros::Duration(5.0).sleep();
      }


      // Replace the start state of the trajectory with the current joint state
      trajectory_msgs::JointTrajectoryPoint start_traj_point;
      start_traj_point.time_from_start = ros::Duration(0.0);
      start_traj_point.positions = joint_state_1.positions;
      start_traj_point.velocities = std::vector<double>(start_traj_point.positions.size(), 0);
      start_traj_point.accelerations = std::vector<double>(start_traj_point.positions.size(), 0);
      start_traj_point.effort = std::vector<double>(start_traj_point.positions.size(), 0);
      goal_msg.trajectory.points[0] = start_traj_point;

      // Send joint trajectory to robot
      ROS_INFO("Please pay attention! Moving the arm!");
      // TODO: merge into sendGoalAndWait()
      // TODO: modify into switch case form to consider all outcomes
      motion_execution_client_.sendGoal(goal_msg);
      motion_execution_client_.waitForResult(ros::Duration(goal_msg.trajectory.points.back().time_from_start.sec) * 1.5);

      if (motion_execution_client_.getState() != actionlib::SimpleClientGoalState::LOST)
      {
        ROS_INFO("Motion execution succeeded!");
        empResponse.success = true;
        empResponse.message = "Motion execution succeeded!";
      }
      else
      {
        ROS_ERROR("Motion execution failed!");
        empResponse.success = false;
        empResponse.message = "Motion execution failed!";
      }
    }
    catch(const std::exception& ex)
    {
      empResponse.success = false;
      empResponse.message = ex.what();
    }
    return empResponse.success;
  }

  private:
  ros::NodeHandle nh_;
  // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> motion_execution_client_;
  ros::ServiceServer motion_execution_server_;

  trajectory_msgs::JointTrajectoryPoint getJointState()
  {
    auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(STATE_FEEDBACK_TOPIC);
    trajectory_msgs::JointTrajectoryPoint joint_state;
    for (int i = 0; i < DOF; i++)
    {
      joint_state.positions.push_back(feedback->actuators[i].position / (180/M_PI));
    }
    return joint_state;
  }


};

int main(int argc, char** argv)
{
  ROS_INFO("Initializing motion exection node!");
  // TODO: to use const string
  ros::init(argc, argv, "gen3_motion_execution_node");
  ros::NodeHandle n;

  MotionExecutionServer gen3_motion_executor(n);
  ros::ServiceServer gen3_motion_execution_server = n.advertiseService(MOTION_EXECUTION_SERVICE, &MotionExecutionServer::executionMotionPlan, &gen3_motion_executor);

  ROS_INFO("Ready to execute motion for Kinova Gen3!");


  ros::spin();

  return 0;
}