#include <kortex_motion_planning/gen3_motion_planner.h> 
#include <kortex_motion_planning/gen3_motion_executor.h>
#include <ros/ros.h>

// todo: move all static constant variable declarations to a new header
// todo: create a common header for both motion planning and execution

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
    Gen3MotionExecutor gen3_motion_executor(n);

    trajectory_msgs::JointTrajectoryPoint joint_state = gen3_motion_executor.getJointState();
    for (size_t i = 0; i < 7; i++)
    {
      current_joint_position(i) = joint_state.positions[i];
    }
    trajectory_msgs::JointTrajectory motion_plan;
    motion_plan = gen3_motion_planner.createGen3MotionPlan(
      urdf_xml_string,
      srdf_xml_string,
      current_joint_position,
      target_translation,
      target_quaternion
    );

    ROS_INFO("Generated trajectory for Kinova Gen3!");

    gen3_motion_executor.executionMotionPlan(motion_plan);

    ros::spin();
    return 0;
}

