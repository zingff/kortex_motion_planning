#include <kortex_motion_planning/simple_jmpe.hpp>

SimpleJmpe::SimpleJmpe(ros::NodeHandle nh_, std::string planning_group_)
{
    planning_group_name_ = planning_group_;
    this->gripper_command_pub_ = nh_.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);

    // Initialize the move group pointer and the joint_positions_ vector
    this->move_group_ptr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
    this->joint_positions_.resize(7); // Assuming 7 joint positions
}

void SimpleJmpe::getCurrentPositions()
{
    // Ensure move_group_ptr_ is initialized
    if (move_group_ptr_ && move_group_ptr_->getCurrentState())
    {
        const robot_state::JointModelGroup *joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup(planning_group_name_);
        current_state_ = move_group_ptr_->getCurrentState();
        current_state_->copyJointGroupPositions(joint_model_group, joint_positions_);
    }
    else
    {
        ROS_ERROR("Move group pointer is not initialized");
    }
}

// Rest of your methods...

