#include <kortex_motion_planning/simple_mpe.hpp>


SimpleMpe::SimpleMpe(ros::NodeHandle nh_, std::string planning_group_)
{
    planning_group_name_ = planning_group_;
    this->gripper_command_pub_ = nh_.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);
    this->joint_positions_.resize(7);
    this->display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
}

void SimpleMpe::getCurrentPositions()
{
    if (move_group_ptr_ && move_group_ptr_->getCurrentState())
    {
        const robot_state::JointModelGroup *joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup(planning_group_name_);
        current_state_ = move_group_ptr_->getCurrentState();
        current_state_->copyJointGroupPositions(joint_model_group, joint_positions_);
        ROS_INFO(GREEN "Move group pointer is initialized" RESET);
    }
    else
    {
        ROS_INFO(RED "Move group pointer is not initialized" RESET);
    }
}

bool SimpleMpe::moveToJoint(const std::vector<double>& joint_positions) {
    try {
        ROS_INFO(WHITE "Ready to move to joint positions" RESET);
        move_group_ptr_->setJointValueTarget(joint_positions);
        
        // Plan
        bool plan_success = (move_group_ptr_->plan(joint_motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!plan_success) {
            ROS_INFO(RED "Joint planning failed" RESET);
            return false;
        }

        // Display planned trajectory (optional)
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = joint_motion_plan_.start_state_;
        display_trajectory.trajectory.push_back(joint_motion_plan_.trajectory_);
        display_publisher_.publish(display_trajectory);

        // Execute
        moveit::planning_interface::MoveItErrorCode execution_result = move_group_ptr_->execute(joint_motion_plan_);
        bool execution_success = (execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (execution_success) {
            ROS_INFO(GREEN "Joint execution successful" RESET);
        } else {
            ROS_INFO(RED "Joint execution failed" RESET);
        }

        return plan_success && execution_success;
    } catch (const std::exception& e) {
        ROS_INFO(RED "Exception during move to joint positions: %s" RESET, e.what());
        return false;
    }
}


void SimpleMpe::setJointGroup(double joint_0, 
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


bool SimpleMpe::moveToCartesian(geometry_msgs::Pose target_pose){
    this->move_group_ptr_->setPoseTarget(target_pose);
    this->move_group_ptr_->setGoalPositionTolerance(0.01);
    this->move_group_ptr_->setGoalOrientationTolerance(0.02);
    this->move_group_ptr_->setPlanningTime(5.0);
    this->move_group_ptr_->setNumPlanningAttempts(5);

    bool plan_success = (this->move_group_ptr_->plan(this->cartesian_motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!plan_success) {
        ROS_INFO(RED "Cartesian planning failed" RESET);
        return plan_success;
    }

    moveit::planning_interface::MoveItErrorCode execution_result = this->move_group_ptr_->execute(this->cartesian_motion_plan_);
    bool execution_success = false;
    if (execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        execution_success = true;
        ROS_INFO(GREEN "Cartesian execution successful" RESET);
    } else {
        ROS_INFO(RED "Cartesian execution failed" RESET);
    } 
    return plan_success && execution_success;
}


// modify to service
void SimpleMpe::goTop()
{
    getCurrentPositions();
    ROS_INFO("Moving to Top Position");
    setJointGroup(0.0212474, 6.01921-6.28, 3.15111, 4.13476-6.28, 0.0608379, 5.37321-6.28, 1.58046);
    moveToJoint(joint_positions_);
}

// Gripper Action Commands, TODO: replace with gripper planning group
void SimpleMpe::closeGripper()
{
    this->gripper_cmd_.goal.command.position = 0.8;
    gripper_command_pub_.publish(gripper_cmd_);
    ROS_WARN("Closing gripper...");
}

void SimpleMpe::openGripper()
{
    this->gripper_cmd_.goal.command.position = 0;
    gripper_command_pub_.publish(gripper_cmd_);
    ROS_WARN("Opening gripper...");
}

bool SimpleMpe::kortexSimpleJointMotionPlanningAndExecution(
  kortex_motion_planning::KortexSimpleJmpe::Request &ksjmpeRequest,
  kortex_motion_planning::KortexSimpleJmpe::Response &ksjmpeResponse
)
{
    ksjmpeResponse.success = false;
    setJointGroup(ksjmpeRequest.target_positions.joint_positions[0], 
                  ksjmpeRequest.target_positions.joint_positions[1], 
                  ksjmpeRequest.target_positions.joint_positions[2], 
                  ksjmpeRequest.target_positions.joint_positions[3], 
                  ksjmpeRequest.target_positions.joint_positions[4], 
                  ksjmpeRequest.target_positions.joint_positions[5], 
                  ksjmpeRequest.target_positions.joint_positions[6]
    );
    moveToJoint(joint_positions_);
    ROS_INFO(GREEN "Joint motion planning and execution succeeded!" RESET);
    ksjmpeResponse.success = true;
    return true;
}


bool SimpleMpe::kortexSimpleCartesianMotionPlanningAndExecution(
  kortex_motion_planning::KortexSimpleCmpe::Request &kscmpeRequest, 
  kortex_motion_planning::KortexSimpleCmpe::Response &kscmpeResponse
)
{
  target_pose_.position = kscmpeRequest.target_pose.position;
  target_pose_.orientation = kscmpeRequest.target_pose.orientation;
  bool success;
  success = moveToCartesian(target_pose_);
  if (success)
  {
    ROS_INFO(GREEN "Cartesian motion planning and execution succeeded!" RESET);
  }
  else
  {
    ROS_INFO(RED "Cartesian motion planning and execution succeeded!" RESET);
  }
  kscmpeResponse.success = success;
  return success;
}