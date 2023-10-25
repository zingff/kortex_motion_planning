// #include <kortex_motion_planning/planning_server.h> 
#include <kortex_motion_planning/gen3_motion_planner.h> 
#include <ros/ros.h>
#include <ros/package.h>
int main(int argc, char** argv)
{
    ROS_INFO("Initializing planning server!");
    // TODO: to use const string
    ros::init(argc, argv, "gen3_motion_planning_node");
    ros::NodeHandle n;
    std::string urdf_xml_string, srdf_xml_string;
    Eigen::VectorXd current_joint_position(7);
    Eigen::Translation3d target_translation;
    Eigen::Quaterniond target_quaternion;
    for (size_t i = 0; i < 7; i++)
    {
      current_joint_position(i) = 0;
    }
    target_translation.x() = -0.15135;
    target_translation.y() = 0.235484;
    target_translation.z() = 0.557796;

    target_quaternion.x() = 0.3872724;
    target_quaternion.y() = -0.4914169;
    target_quaternion.z() = -0.604657;
    target_quaternion.w() = 0.4928685;

    
    n.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    n.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);


    Gen3MotionPlanner gen3_motion_planner;
    trajectory_msgs::JointTrajectory motion_plan;
    motion_plan = gen3_motion_planner.createGen3MotionPlan(
      urdf_xml_string,
      srdf_xml_string,
      current_joint_position,
      target_translation,
      target_quaternion
    );

    ROS_INFO("Ready to provide trajectory for Kinova Gen3!");

    ros::spin();
    return 0;
}

