#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <kortex_motion_planning/GenerateKortexMotionPlan.h>
#include <kortex_motion_planning/ExecuteMotionPlan.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <Eigen/Geometry>

static const std::string PLANNING_SERVICE = "/my_gen3/motion_planning_server";
static const std::string MOTION_EXECUTION_SERVICE = "/my_gen3/motion_execution_server";
static const int DOF = 7;
static const std::string STATE_FEEDBACK_TOPIC = "/my_gen3/base_feedback";

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
    auto stateFeedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(STATE_FEEDBACK_TOPIC);
    double roll = stateFeedback->base.tool_pose_theta_x / (180 / M_PI);
    double pitch = stateFeedback->base.tool_pose_theta_y / (180 / M_PI);    
    double yaw = stateFeedback->base.tool_pose_theta_z / (180 / M_PI);
    roll = roll - M_PI;
    pitch = pitch - M_PI;

    Eigen::Vector3d euler(roll, pitch, yaw);
    Eigen::AngleAxisd rotation(M_PI / 1, Eigen::Vector3d::UnitZ());
    euler = rotation.toRotationMatrix() * euler;
    std::cout << euler << std::endl;

    double x = stateFeedback->base.tool_pose_x;
    double y = stateFeedback->base.tool_pose_y;
    double z = stateFeedback->base.tool_pose_z;

    Eigen::Quaterniond target_quaternion;
    target_quaternion = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
              // * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    std::cout << target_quaternion.x() << ", " 
              << target_quaternion.y() << ", "
              << target_quaternion.z() << ", "
              << target_quaternion.w() << std::endl;
// -0.15135, 0.235484, 0.557796, -115.418, 178.321, 80.9466

    target_pose_.position.x = -0.15135;
    target_pose_.position.y = 0.235484;
    target_pose_.position.z = 0.557796;
    // target_pose_.position.x = x;
    // target_pose_.position.y = y;
    // target_pose_.position.z = z;
    // 0.2497374, 0.8497921, 0.2278447, 0.4044396
// 0.3372995, 0.4143812, -0.6379122, 0.5546038
    target_pose_.orientation.x = 0.3372995;
    target_pose_.orientation.y = 0.4143812;
    target_pose_.orientation.z = -0.6379122;
    target_pose_.orientation.w = 0.5546038;
    // target_pose_.orientation.x = target_quaternion.x();
    // target_pose_.orientation.y = target_quaternion.y();
    // target_pose_.orientation.z = target_quaternion.z();
    // target_pose_.orientation.w = target_quaternion.w();
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
    // for (int i = 0; i <motion_plan_response_.motion_plan.points.size(); i++)
    // {
    //   for (int j = 0; j < DOF; j++)
    //   {
    //     std::cout << motion_plan_response_.motion_plan.points[i].positions[j] << std::endl;
    //   }
    // }
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

  ROS_WARN("Please check the motion plan in Rivz!");
  int decision;
  std::cout << "1 - execute, 0 - abort" <<std::endl;
  std::cin >> decision;
  if (decision == 1)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    KMT.execute();
    ROS_INFO("Going to execute motion, attention!");

  }
  else if (decision == 0)
  {
    ROS_INFO("Abort to execute motion plan!");
  }
  return 0;
}