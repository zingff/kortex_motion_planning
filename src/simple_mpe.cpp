#include <kortex_motion_planning/simple_mpe.hpp>


SimpleMpe::SimpleMpe(ros::NodeHandle nh_, std::string planning_group_)
{
    planning_group_name_ = planning_group_;
    this->gripper_command_pub_ = nh_.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);
    this->joint_positions_.resize(7);
    this->display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
    twist_force_z_publisher_ = nh_.advertise<std_msgs::Float64>("/wrench_force_z", 10);
    this->gripper_cmd_client_ = nh_.serviceClient<kortex_driver::SendGripperCommand>("/base/send_gripper_command");
    this->twist_command_client_ = nh_.serviceClient<kortex_driver::SendTwistCommand>("/base/send_twist_command");
    getROSParam(nh_, "/foodSkewer/uprightSkewerTwist", upright_skewer_twist_);
    getROSParam(nh_, "/foodSkewer/zeroSkewerTwist", zero_twist_);
    getROSParam(nh_, "/foodSkewer/lpfCoefficient", lpf_coefficient_);
    getROSParam(nh_, "/foodSkewer/wrenchForceZThreshold", wrench_force_z_threshold_);
    getROSParam(nh_, "/kortex/service/sendGripperCommand", send_gripper_command_service_);
    getROSParam(nh_, "/kortex/service/sendTwistCommand", send_twist_command_service_);
    getROSParam(nh_, "/kortex/topic/baseCyclicFeedback", base_cyclic_feedback_topic_);
    getROSParam(nh_, "/kortex/config/baseFrame", reference_frame_);
    getROSParam(nh_, "/rosConfig/simpleCMPEConfig/goalPositionTorlerance", goal_position_tolerance_);
    getROSParam(nh_, "/rosConfig/simpleCMPEConfig/planningAttemptsNumber", planning_attempts_number_);
    getROSParam(nh_, "/rosConfig/simpleCMPEConfig/planningTime", planning_time_);
    getROSParam(nh_, "/rosConfig/simpleCMPEConfig/goalOrientationTorlerance", goal_orientation_tolerance_);

    getROSParam(nh_, "/utensil/holderPosition", holder_positions_);
    getROSParam(nh_, "/utensil/utensilPosition", utensil_positions);


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
        ros::Duration(0.1).sleep();
    
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
    this->move_group_ptr_->setGoalPositionTolerance(goal_position_tolerance_);
    this->move_group_ptr_->setGoalOrientationTolerance(goal_orientation_tolerance_);
    this->move_group_ptr_->setPlanningTime(planning_time_);
    this->move_group_ptr_->setNumPlanningAttempts(planning_attempts_number_);
    // todo: modify  vel, acc, etc., params with ros param server
    this->move_group_ptr_->setMaxAccelerationScalingFactor(0.5);

    bool plan_success = (this->move_group_ptr_->plan(this->cartesian_motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!plan_success) {
        ROS_INFO(RED "Cartesian planning failed" RESET);
        return plan_success;
    }

    moveit::planning_interface::MoveItErrorCode execution_result = this->move_group_ptr_->execute(this->cartesian_motion_plan_);
    ros::Duration(0.1).sleep();
    bool execution_success = false;
    if (execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        execution_success = true;
        ROS_INFO(GREEN "Cartesian execution successful" RESET);
    } else {
        ROS_INFO(RED "Cartesian execution failed" RESET);
    } 
    return plan_success && execution_success;
}


void SimpleMpe::moveToFeedingInitialPositions()
{
    getCurrentPositions();
    ROS_INFO(GREEN "Moving to feeding initial position" RESET);
    setJointGroup(0.0212474, 6.01921-6.28, 3.15111, 4.13476-6.28, 0.0608379, 5.37321-6.28, 1.58046);
    moveToJoint(joint_positions_);
}

// Gripper Action Commands
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

void SimpleMpe::sendGripperCommand(double gripper_position_)
{
  kortex_driver::Finger finger;
  kortex_driver::SendGripperCommand send_girpper_command;
  finger.finger_identifier = 0;
  finger.value = gripper_position_;
  send_girpper_command.request.input.gripper.finger.push_back(finger);
  send_girpper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

  if (gripper_cmd_client_.call(send_girpper_command))
  {
    ROS_INFO(GREEN "Sending gripper command: %f." RESET, gripper_position_);
  }
  else
  {
    ROS_INFO(RED "Failed to call service SendGripperCommand!");
  }
  
}

bool SimpleMpe::sendKortexGripperCommand(
  kortex_motion_planning::SendGripperCommandRequest &skgcRequest,
  kortex_motion_planning::SendGripperCommandResponse &skgcResponse
)
{
  skgcResponse.success = false;
  double gripper_position;
  gripper_position = skgcRequest.gripper_position;
  sendGripperCommand(gripper_position);
  skgcResponse.success = true;
}

bool SimpleMpe::getUtensil(
  kortex_motion_planning::GetUtensilRequest &guRequest,
  kortex_motion_planning::GetUtensilResponse &guResponse
)
{
  guResponse.success = false;
  if (guRequest.get_utensil_flag = true)
  {
    moveToFeedingInitialPositions();
    ros::Duration(0.2).sleep();
    sendGripperCommand(0);
    ros::Duration(0.2).sleep();
    ROS_INFO(GREEN "Ready to move to utensil position." RESET);
    setJointGroup(guRequest.holder_positions.joint_positions[0], 
                  guRequest.holder_positions.joint_positions[1], 
                  guRequest.holder_positions.joint_positions[2], 
                  guRequest.holder_positions.joint_positions[3], 
                  guRequest.holder_positions.joint_positions[4], 
                  guRequest.holder_positions.joint_positions[5], 
                  guRequest.holder_positions.joint_positions[6]
    );
    moveToJoint(joint_positions_);
    ros::Duration(0.5).sleep();
    sendGripperCommand(1.0);
    ros::Duration(0.2).sleep();
    setJointGroup(guRequest.utensil_positions.joint_positions[0], 
                  guRequest.utensil_positions.joint_positions[1], 
                  guRequest.utensil_positions.joint_positions[2], 
                  guRequest.utensil_positions.joint_positions[3], 
                  guRequest.utensil_positions.joint_positions[4], 
                  guRequest.utensil_positions.joint_positions[5], 
                  guRequest.utensil_positions.joint_positions[6]
    );
    moveToJoint(joint_positions_);
    ros::Duration(0.2).sleep();
    guResponse.success = true;
  }
}


bool SimpleMpe::getUtensilAction()
{
  moveToFeedingInitialPositions();
  ros::Duration(0.2).sleep();
  sendGripperCommand(0);
  ros::Duration(0.2).sleep();
  ROS_INFO(GREEN "Ready to move to utensil position." RESET);
  setJointGroup(holder_positions_[0], 
                holder_positions_[1], 
                holder_positions_[2], 
                holder_positions_[3], 
                holder_positions_[4], 
                holder_positions_[5], 
                holder_positions_[6]
  );
  moveToJoint(joint_positions_);
  ros::Duration(0.5).sleep();
  sendGripperCommand(1.0);
  ros::Duration(0.2).sleep();
  setJointGroup(utensil_positions[0], 
                utensil_positions[1], 
                utensil_positions[2], 
                utensil_positions[3], 
                utensil_positions[4], 
                utensil_positions[5], 
                utensil_positions[6]
  );
  moveToJoint(joint_positions_);
  ros::Duration(0.2).sleep(); 
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
  ros::Duration(0.1).sleep();
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

bool SimpleMpe::uprightSkewerAction(
  kortex_motion_planning::UprightSkewerActionRequest &usaRequest,
  kortex_motion_planning::UprightSkewerActionResponse &usaResponse
)
{
  usaResponse.success = false;
  double current_wrench_force_z = 0;
  double previous_wrench_force_z = 0;
  double delta_wrench_force_z = 0;
  bool success = false;
  if (usaRequest.skewer_action_flag == true)
  {
    int i = 0;
    twistCommand(upright_skewer_twist_, reference_frame_);
    while (ros::ok())
    {
      auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/base_feedback");
      current_wrench_force_z = lpf_coefficient_*feedback->base.tool_external_wrench_force_z + (1 - lpf_coefficient_) * previous_wrench_force_z;
      delta_wrench_force_z = std::abs(current_wrench_force_z - previous_wrench_force_z);
      previous_wrench_force_z = current_wrench_force_z;
      i++;
      ROS_INFO(WHITE "Wrench force Z: %f" RESET, current_wrench_force_z);

      if ((current_wrench_force_z >= wrench_force_z_threshold_) && (i>2))
      {
        ROS_INFO(YELLOW "Reached the threshold of the skewer force!" RESET);
        twistCommand(zero_twist_, reference_frame_);
        usaResponse.success = true;
        break;
      }
      ros::spinOnce();
    }
    
  }
  return usaResponse.success;
}