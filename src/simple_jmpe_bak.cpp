#include <kortex_motion_planning/simple_jmpe.hpp>


SimpleJmpe::SimpleJmpe(ros::NodeHandle nh_, std::string planning_group_)
{
    planning_group_name_ = planning_group_;
    this->gripper_command_pub_ = nh_.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);
    this->joint_positions_.resize(7);
    this->display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
}

void SimpleJmpe::getCurrentPositions()
{
    if (move_group_ptr_ && move_group_ptr_->getCurrentState())
    {
        const robot_state::JointModelGroup *joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup(planning_group_name_);
        current_state_ = move_group_ptr_->getCurrentState();
        current_state_->copyJointGroupPositions(joint_model_group, joint_positions_);
        ROS_INFO("Move group pointer is initialized");
    }
    else
    {
        ROS_ERROR("Move group pointer is not initialized");
    }
}

void SimpleJmpe::moveToJoint(std::vector<double>)
{
    try {
        ROS_INFO("Ready to move to joint positions");
        move_group_ptr_->setJointValueTarget(joint_positions_);
        bool success = (move_group_ptr_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            // Output the waypoints
            const auto& points = motion_plan_.trajectory_.joint_trajectory.points;
            for (size_t i = 0; i < points.size(); ++i) {
                std::stringstream ss;
                ss << "Waypoint " << i << ": [";
                for (size_t j = 0; j < points[i].positions.size(); ++j) {
                    ss << points[i].positions[j];
                    if (j < points[i].positions.size() - 1) ss << ", ";
                }
                ss << "]";
                ROS_INFO_STREAM(ss.str());
            }

            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = motion_plan_.start_state_;
            display_trajectory.trajectory.push_back(motion_plan_.trajectory_);
            display_publisher_.publish(display_trajectory);
            move_group_ptr_->move();
            ROS_INFO("Move completed");
        } else {
            ROS_ERROR("Motion planning failed");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during move to joint positions: %s", e.what());
    }
}

void SimpleJmpe::setJointGroup(double joint_0, 
                               double joint_1, 
                               double joint_2,
                               double joint_3, 
                               double joint_4, 
                               double joint_5, 
                               double joint_6
)
{
    joint_positions_[0] = joint_0;
    joint_positions_[1] = joint_1;
    joint_positions_[2] = joint_2;
    joint_positions_[3] = joint_3;
    joint_positions_[4] = joint_4;
    joint_positions_[5] = joint_5;
    joint_positions_[6] = joint_6;
}

// modify to service
void SimpleJmpe::goTop()
{
    getCurrentPositions();
    ROS_INFO("Moving to Top Position");
    setJointGroup(0.0212474, 6.01921-6.28, 3.15111, 4.13476-6.28, 0.0608379, 5.37321-6.28, 1.58046);
    moveToJoint(joint_positions_);
}

// Gripper Action Commands, TODO: replace with gripper planning group
void SimpleJmpe::closeGripper()
{
    this->gripper_cmd_.goal.command.position = 0.8;
    gripper_command_pub_.publish(gripper_cmd_);
    ROS_WARN("Closing gripper...");
}

void SimpleJmpe::openGripper()
{
    this->gripper_cmd_.goal.command.position = 0;
    gripper_command_pub_.publish(gripper_cmd_);
    ROS_WARN("Opening gripper...");
}

bool SimpleJmpe::kortexSimpleJointMotionPlanningAndExecution(kortex_motion_planning::KortexSimpleJmpe::Request &ksjmpeRequest,
                              kortex_motion_planning::KortexSimpleJmpe::Response &ksjmpeResponse)
{
    setJointGroup(ksjmpeRequest.target_positions.joint_positions[0], 
                  ksjmpeRequest.target_positions.joint_positions[1], 
                  ksjmpeRequest.target_positions.joint_positions[2], 
                  ksjmpeRequest.target_positions.joint_positions[3], 
                  ksjmpeRequest.target_positions.joint_positions[4], 
                  ksjmpeRequest.target_positions.joint_positions[5], 
                  ksjmpeRequest.target_positions.joint_positions[6]
    );
    moveToJoint(joint_positions_);
    ROS_INFO("Motion planning and execution succeeded!");
    ksjmpeResponse.success = true;
    return true;
}

int main(int argc, char **argv)
{
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    ros::init(argc, argv, "simple_jmpe_service_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::string planning_group = "arm";
    SimpleJmpe simple_jmpe(nh, planning_group);

    simple_jmpe.planning_scene_ptr_ = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface()
    );
    // simple_jmpe.move_group_ptr_ = MoveGroupPtr(
    //   new moveit::planning_interface::MoveGroupInterface(
    //     simple_jmpe.planning_group_name_
    //   )
    // );
    simple_jmpe.move_group_ptr_.reset(new moveit::planning_interface::MoveGroupInterface(
        simple_jmpe.planning_group_name_
      ));

    ros::ServiceServer service = nh.advertiseService("kortex_simple_joint_motion_service", &SimpleJmpe::kortexSimpleJointMotionPlanningAndExecution, &simple_jmpe);
    ROS_INFO("Kortex Simple Joint Motion service initialized.");

    ros::waitForShutdown();
    return 0;
}