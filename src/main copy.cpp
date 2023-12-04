#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <kortex_motion_planning/GenerateKortexMotionPlan.h>
#include <kortex_motion_planning/ExecuteMotionPlan.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>

static const std::string PLANNING_SERVICE = "/motion_planning_server";
static const std::string MOTION_EXECUTION_SERVICE = "/motion_execution_server";
static const int DOF = 7;
static const std::string STATE_FEEDBACK_TOPIC = "/base_feedback";

class KortexMotionWidget
{
  public:
  KortexMotionWidget()
  {
    motion_planning_client_ = nh_.serviceClient<kortex_motion_planning::GenerateKortexMotionPlan>(PLANNING_SERVICE);
    motion_execution_client_ = nh_.serviceClient<kortex_motion_planning::ExecuteMotionPlan>(MOTION_EXECUTION_SERVICE);
  }
  void plan()
  {
    target_pose_.position.x = 0.420817;
    target_pose_.position.y = -0.142423;
    target_pose_.position.z = 0.151256;
    target_pose_.orientation.x = -0.627236;
    target_pose_.orientation.y = -0.591028;
    target_pose_.orientation.z = -0.348967;
    target_pose_.orientation.w =  0.368081;

    tf2::Quaternion quaternion;
    quaternion.setX(target_pose_.orientation.x);
    quaternion.setY(target_pose_.orientation.y);
    quaternion.setZ(target_pose_.orientation.z);
    quaternion.setW(target_pose_.orientation.w);
    quaternion.normalize();

    target_pose_.orientation.x = quaternion.getX();
    target_pose_.orientation.y = quaternion.getY();
    target_pose_.orientation.z = quaternion.getZ();
    target_pose_.orientation.w = quaternion.getW();

    std::cout << "normalized quaternion:" << std::endl;
    std::cout << "x: " << target_pose_.orientation.x << std::endl;
    std::cout << "y: " << target_pose_.orientation.y << std::endl;
    std::cout << "z: " << target_pose_.orientation.z << std::endl;
    std::cout << "w: " << target_pose_.orientation.w << std::endl;

    motion_planning_service_.request.target_pose = target_pose_;
    if (motion_planning_client_.call(motion_planning_service_.request, motion_planning_service_.response))
    {
      ROS_INFO("Succeeded to call kortex motion planning service!");
      motion_plan_response_ = motion_planning_service_.response;
      ROS_INFO("Succeeded to store kortex motion planning result!");
    }
    else
    {
      ROS_ERROR("Failed to call kortex motion planning service!");
      exit(1);
    }
  }

  void execute()
  {
    motion_execution_service_.request.motion_plan = motion_plan_response_.motion_plan;
    if (motion_execution_client_.call(motion_execution_service_))
    {
      ROS_INFO("Succeeded to call kortex motion execution service!");
    }
    else
    {
      ROS_INFO("Failed to call kortex motion execution service!");
      exit(1);
    }
  }

  private:
  ros::NodeHandle nh_;
  geometry_msgs::Pose target_pose_;
  ros::ServiceClient motion_planning_client_;
  ros::ServiceClient motion_execution_client_;
  std::shared_ptr<trajectory_msgs::JointTrajectory> motion_plan_{ nullptr };
  kortex_motion_planning::GenerateKortexMotionPlan motion_planning_service_;
  kortex_motion_planning::ExecuteMotionPlan motion_execution_service_;
  kortex_motion_planning::GenerateKortexMotionPlanResponse motion_plan_response_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kortex_motion_widget");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::Pose target_pose;  

  KortexMotionWidget KMT;
  ROS_INFO("Kortex planning motion widget initialized!");
  auto start_time = std::chrono::system_clock::now();
  auto start_time_stamp = std::chrono::system_clock::to_time_t(start_time);
  ROS_INFO("Motion planning started finished at %s", std::ctime(&start_time_stamp));

  KMT.plan();

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = end_time - start_time;
  std::time_t end_time_stamp = std::chrono::system_clock::to_time_t(end_time);

  ROS_INFO("Motion planning finished at %s", std::ctime(&end_time_stamp));
  ROS_INFO("Elapsed time for motion planning: %f seconds", elapsed_time.count());

  KMT.execute();

  return 0;
}
