#include <kortex_motion_planning/simple_mpe.hpp>

int main(int argc, char **argv)
{
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    ros::init(argc, argv, "simple_mpe_service_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::string planning_group = "arm";
    SimpleMpe simple_mpe(nh, planning_group);

    simple_mpe.planning_scene_ptr_ = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface()
    );
    // simple_jmpe.move_group_ptr_ = MoveGroupPtr(
    //   new moveit::planning_interface::MoveGroupInterface(
    //     simple_jmpe.planning_group_name_
    //   )
    // );
    simple_mpe.move_group_ptr_.reset(new moveit::planning_interface::MoveGroupInterface(
        simple_mpe.planning_group_name_
      ));

    ros::ServiceServer jmpe_service = nh.advertiseService(
      "kortex_simple_joint_motion_service", 
      &SimpleMpe::kortexSimpleJointMotionPlanningAndExecution, 
      &simple_mpe
    );
    ROS_INFO(WHITE "Kortex Simple Joint Motion service initialized!" RESET);

    ros::ServiceServer cmpe_service = nh.advertiseService(
      "kortex_simple_cartesian_motion_service",
      &SimpleMpe::kortexSimpleCartesianMotionPlanningAndExecution,
      &simple_mpe
    );
    ROS_INFO(WHITE "Kortex Simple Cartesian Motion service initialized!" RESET);

    ros::waitForShutdown();
    return 0;
}