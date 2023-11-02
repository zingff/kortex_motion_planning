#ifndef GEN3_MOTION_PLANNER_H
#define GEN3_MOTION_PLANNER_H

#include <kortex_motion_planning/planner_profiles.hpp>
#include <console_bridge/console.h>
#include <tesseract_common/timer.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/profiles/min_length_profile.h>
#include <tesseract_task_composer/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/task_composer_problem.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_plugin_factory.h>
#include <tesseract_support/tesseract_support_resource_locator.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit/robot_model/robot_model.h>
#include <kortex_motion_planning/GenerateKortexMotionPlan.h>

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "GEN3";
static const std::string PROFILE2 = "GEN3_FREESPACE";
static const std::string PLANNING_SERVICE = "/my_gen3/motion_planning_server";
static const std::string MONITOR_NAMESPACE = "gen3_environment";
static const std::string TASK_PIPELINE = "Gen3Pipeline";
// static const std::string STATE_FEEDBACK_TOPIC = "/my_gen3/base_feedback";
static const std::string STATE_FEEDBACK_TOPIC = "/base_feedback";

static const double MAX_TCP_SPEED = 0.2;
const std::string ROBOT_DESCRIPTION_PARAM = "trajopt_description";
const std::string ROBOT_SEMANTIC_PARAM = "trajopt_description_semantic";
static const int DOF = 7;
static const std::string MANIPULATOR_GROUP_NAME = "manipulator";
static const std::string WORKING_FRAME_NAME = "base_link";
static const std::string TCP_FRAME_NAME = "tool_frame";
static const std::string TASK_COMPOSER_PLUGIN_PKG_NAME = "kortex_motion_planning";
static const std::string TASK_COMPOSER_PLUGIN_SUB_PATH = "/tesseract/task_composer_plugins.yaml";
static const std::string URDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.urdf";
static const std::string SRDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.srdf";

class Gen3MotionPlanner
{
public:
  Gen3MotionPlanner();
  trajectory_msgs::JointTrajectory createGen3MotionPlan(
    const std::string urdf_xml_string,
    const std::string srdf_xml_string,
    Eigen::VectorXd current_joint_position,
    Eigen::Translation3d target_translation,
    Eigen::Quaterniond target_quaternion
  );

private:
  tesseract_common::JointTrajectory joint_trajectory_;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  moveit::core::RobotModelPtr robot_model_;

  tesseract_environment::Environment::Ptr env_;
  tesseract_planning::ProfileDictionary::Ptr profile_dictionary_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_visualization::Visualization::Ptr plotter_;

  Eigen::VectorXd current_joint_position_;
  std::vector<std::string> joint_names_;
  tesseract_common::ManipulatorInfo manipulator_info_;

  trajectory_msgs::JointTrajectory motion_plan_;

  void getCurrentPosition();
  tesseract_common::JointTrajectory tcpSpeedLimiter(
      const tesseract_common::JointTrajectory &input_trajectory,
      const double max_speed,
      const std::string tcp);
  tesseract_planning::CompositeInstruction createProgram(
      const tesseract_common::ManipulatorInfo &info,
      const Eigen::Isometry3d &end_pose);
};

Gen3MotionPlanner::Gen3MotionPlanner()
: env_(std::make_shared<tesseract_environment::Environment>())
{
  
}

trajectory_msgs::JointTrajectory Gen3MotionPlanner::createGen3MotionPlan(
    const std::string urdf_xml_string,
    const std::string srdf_xml_string,
    Eigen::VectorXd current_joint_position,
    Eigen::Translation3d target_translation,
    Eigen::Quaterniond target_quaternion
) // : env_(std::make_shared<tesseract_environment::Environment>())
{ 
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  // Create robot model 
  CONSOLE_BRIDGE_logInform("Ready to create robot model!");
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  // auto locator = std::make_shared<TesseractSupportResourceLocator>();

  tesseract_common::fs::path urdf_path = locator->locateResource("package://" + URDF_FILE_PATH)->getFilePath();
  tesseract_common::fs::path srdf_path = locator->locateResource("package://" + SRDF_FILE_PATH)->getFilePath();


  // note: the 2 string are args 
  if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment!");
  }
  CONSOLE_BRIDGE_logInform("Environment initialized!");

  urdf_model_ = urdf::parseURDF(urdf_xml_string);
  CONSOLE_BRIDGE_logInform("URDF parsed!");
  srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model_->initString(*urdf_model_, srdf_xml_string);
  CONSOLE_BRIDGE_logInform("SRDF parsed!");
  assert(urdf_model_ != nullptr && srdf_model_ != nullptr);
  robot_model_.reset(new moveit::core::RobotModel(urdf_model_, srdf_model_));
  CONSOLE_BRIDGE_logInform("Robot model loaded!");

  manipulator_info_ = tesseract_common::ManipulatorInfo(MANIPULATOR_GROUP_NAME, WORKING_FRAME_NAME, TCP_FRAME_NAME);
  std::vector<std::string> joint_names_ = env_->getJointGroup(manipulator_info_.manipulator)->getJointNames();
  CONSOLE_BRIDGE_logInform("Joint names got!");

  // get current position
  env_->setState(joint_names_, current_joint_position);

  // Create environment monitor
  monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env_, MONITOR_NAMESPACE);
  monitor_->setEnvironmentPublishingFrequency(40);
  monitor_->startPublishingEnvironment();
  monitor_->startStateMonitor("/my_gen3/joint_states", true);

  // Create plotter
  // todo: check the logic variable
  if(true)
  {
    // plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getRootLinkName());
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());  // equivalent
    CONSOLE_BRIDGE_logInform("Monitor created!");

    // TODO: here to define the original example class
    profile_dictionary_ = std::make_shared<tesseract_planning::ProfileDictionary>();

    // Add custom profiles
    profile_dictionary_->addProfile<tesseract_planning::SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, PROFILE,
      createSimplePlannerProfile());
    CONSOLE_BRIDGE_logInform("Simple profile added!");

    // profile_dictionary_->addProfile<tesseract_planning::OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, PROFILE, 
    //   createOMPLProfile());
    // CONSOLE_BRIDGE_logInform("OMPL profile added!");

    profile_dictionary_->addProfile<tesseract_planning::TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE, 
      createTrajOptCompositeProfile());
    CONSOLE_BRIDGE_logInform("TrajOpt composite profile added!");

    profile_dictionary_->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptPlanProfile());
    CONSOLE_BRIDGE_logInform("TrajOpt plan profile added!");

    // profile_dictionary_->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptToolZFreePlanProfile());
    // CONSOLE_BRIDGE_logInform("TrajOpt plan profile (z axis rotation-free) added!");

    profile_dictionary_->addProfile<tesseract_planning::DescartesPlanProfile<float>>(
      DESCARTES_DEFAULT_NAMESPACE, PROFILE, 
      createDescartesPlanProfile<float>());
    CONSOLE_BRIDGE_logInform("Descartes profile added!");

    // profile_dictionary_->addProfile<tesseract_planning::MinLengthProfile>(
    //   MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE,
    //   std::make_shared<tesseract_planning::MinLengthProfile>(5));
    // CONSOLE_BRIDGE_logInform("Min length profile added!");

    // profile_dictionary_->addProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
    //   ISP_DEFAULT_NAMESPACE, PROFILE,
    //   std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>());
    // CONSOLE_BRIDGE_logInform("ISP profile added!");

    CONSOLE_BRIDGE_logInform("Plan profile created");
  }

  // note: the following part is from original function plan
  // try
  // {
    CONSOLE_BRIDGE_logInform("Received kortex motion planning request!");

    // TODO: load robot model
    // Create manipulator info
    tesseract_common::ManipulatorInfo manipulator_info(MANIPULATOR_GROUP_NAME, WORKING_FRAME_NAME, TCP_FRAME_NAME);
    CONSOLE_BRIDGE_logInform("Manipulator info created!");

    // Initialize start configuration with current joint positions
    // todo: ros free
    joint_names_ = env_->getJointGroup(manipulator_info.manipulator)->getJointNames();
    CONSOLE_BRIDGE_logInform("Joint names got!");
    // auto stateFeedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(STATE_FEEDBACK_TOPIC);
    Eigen::VectorXd start_configuration(DOF);
    for (int i = 0; i < DOF; i++)
    {
      // double joint_position = stateFeedback->actuators[i].position/(180.0/M_PI);
      double joint_position = current_joint_position(i); // rad
      // std::cout << "The joint position is: " << joint_position << std::endl;
      if ((i == 1 && abs(joint_position) >= 2.41) || 
          (i == 3 && abs(joint_position) >= 2.66) ||
          (i == 5 && abs(joint_position) >= 2.23))
      {
        if (joint_position < 0)
        {
          joint_position += 2 * M_PI;
        }
        else
        {
          joint_position -= 2 * M_PI;
        }
      }
      start_configuration(i) = joint_position;
      std::cout << start_configuration(i) << std::endl;
    }

    // Initialize target configuration
    Eigen::Isometry3d target_configuration;  // target in cartesian

    target_configuration = Eigen::Isometry3d::Identity() * target_translation * target_quaternion;

    // Set initial state in the tesseract environment
    env_->setState(joint_names_, start_configuration);

    // Set up composite instruction and environment
    tesseract_planning::CompositeInstruction program = createProgram(manipulator_info, target_configuration);

    // Set up task composer problem
    std::string config_path = KORTEX_CONFIG_DIR;

    if (config_path.empty())
    {
      CONSOLE_BRIDGE_logError("Failed to get path of config file, please check!");
    }
    config_path += TASK_COMPOSER_PLUGIN_SUB_PATH;
    tesseract_planning::TaskComposerPluginFactory factory(YAML::LoadFile(config_path));
    CONSOLE_BRIDGE_logInform("Task composer plugin factory created!");

    // Create executor      
    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

    // Create task
    // TODO: Gen3Pipeline 1.0 done copy of CartesianPipeline
    // TODO: Gen3Pipeline 2.0 done simple->TrajOpt
    tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(TASK_PIPELINE);
    const std::string input_key = task->getInputKeys().front();
    const std::string output_key = task->getOutputKeys().front();
    CONSOLE_BRIDGE_logInform("Task composer node created!");

    // Save dot graph
    std::ofstream task_composer_data;
    task_composer_data.open(tesseract_common::getTempPath() + TASK_PIPELINE + ".dot");  // /temp
    task->dump(task_composer_data);
    CONSOLE_BRIDGE_logInform("Dot graph saved!");

    // Create task input data
    tesseract_planning::TaskComposerDataStorage input_data;
    input_data.setData(input_key, program);

    // Create task composer problem
    tesseract_planning::TaskComposerProblem problem(env_, input_data);

    // Update log level for debugging
    auto log_level = console_bridge::getLogLevel();
    std::cout << "Log level: " << log_level << std::endl;
    // todo: check if the logical value is needed
    if (false)
    {
      // not currently used 
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

      // Create dump dotgraphs of each task for reference
      std::ofstream cartesian_pipeline_output_data;
      cartesian_pipeline_output_data.open(tesseract_common::getTempPath() + "Gen3Pipeline.dot");
      factory.createTaskComposerNode("Gen3Pipeline")->dump(cartesian_pipeline_output_data);

      // std::ofstream cartesian_pipeline_output_data;
      // cartesian_pipeline_output_data.open(tesseract_common::getTempPath() + "Gen3CartesianPipeline.dot");
      // factory.createTaskComposerNode("Gen3CartesianPipeline")->dump(cartesian_pipeline_output_data);

      // std::ofstream freespace_pipeline_output_data;
      // freespace_pipeline_output_data.open(tesseract_common::getTempPath() + "Gen3FreespacePipeline.dot");
      // factory.createTaskComposerNode("Gen3FreespacePipeline")->dump(freespace_pipeline_output_data);
      // // TODO: check if other pipeline output data is needed
    }

    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    // Solve process plan
    CONSOLE_BRIDGE_logInform("Begin to solve the program!");
    tesseract_common::Timer stopwatch;  // get solving time
    stopwatch.start();
    tesseract_planning::TaskComposerInput input(problem, profile_dictionary_);  // passed as an input to each process in the decision tree
    tesseract_planning::TaskComposerFuture::UPtr executor_future = executor->run(*task, input);
    executor_future->wait();
    stopwatch.stop();
    CONSOLE_BRIDGE_logInform("Planning time: %f seconds", stopwatch.elapsedSeconds());

    // Reset the log level
    // console_bridge::setLogLevel(log_level);

    // Get motion planning result
    tesseract_planning::CompositeInstruction program_result = input.data_storage.getData(output_key).as<tesseract_planning::CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(program_result, *env_);

    // Check for successful plan
    if (!input.isSuccessful())
    {
      CONSOLE_BRIDGE_logError("Failed to create motion plan!");
    }
    else
    {
      CONSOLE_BRIDGE_logInform("Succeeded to create motion plan!");
    }

    // // Display waypoints
    // // TODO: to use CatesianWaypoint.print()
    // CONSOLE_BRIDGE_logInform("Waypoint number: %d", toolpath.data()->size());
    // for (int n_tp = 0; n_tp < toolpath.size(); n_tp++)
    // {
    //   for (int i = 0; i < toolpath[n_tp].size(); i++)
    //   {
    //     std::cout << "Cart WP " << i << ": xyz =" 
    //               << toolpath[n_tp][i].translation().x() << ", "
    //               << toolpath[n_tp][i].translation().y() << ", "
    //               << toolpath[n_tp][i].translation().z()                  
    //               << std::endl;
    //     // Note that the euler order should be 2, 1, 0
    //     std::cout << "        " << i << ": rpy =" 
    //               << toolpath[n_tp][i].rotation().eulerAngles(2, 1, 0)[0] << ", "
    //               << toolpath[n_tp][i].rotation().eulerAngles(2, 1, 0)[1] << ", " 
    //               << toolpath[n_tp][i].rotation().eulerAngles(2, 1, 0)[2]                   
    //               << std::endl;
    //   }
    // }

    joint_trajectory_ = tesseract_planning::toJointTrajectory(program_result);

    // Convert to joint trajectory
    tesseract_common::JointTrajectory joint_trajectory = tesseract_planning::toJointTrajectory(program_result);
    tesseract_common::JointTrajectory tcp_velocity_scaled_joint_trajectory = tcpSpeedLimiter(joint_trajectory, MAX_TCP_SPEED, TCP_FRAME_NAME);

    // Send joint trajectory to tesseract plottor widget
    if (plotter_ != nullptr && plotter_->isConnected())
    {
      // // Hit enter key to continue!
      // plotter_->waitForInput();  
      // Display tool coordinate 
      plotter_->plotMarker(tesseract_visualization::ToolpathMarker(toolpath));
      // Play animation in rviz
      plotter_->plotTrajectory(joint_trajectory, *env_->getStateSolver());
    }

    // Return result
    motion_plan_ = tesseract_rosutils::toMsg(tcp_velocity_scaled_joint_trajectory, env_->getState());
    // res.message = "Motion planning succeeded!";
    // res.success = true;
    // }
    // catch (const std::exception& ex)
    // {
    //   // todo: deal with failed plan
    //   // res.message = ex.what();
    //   // res.success = false;
    //   // return false;
    // }
    std::cout << motion_plan_.points[1].time_from_start << std::endl;
    return motion_plan_;

}

tesseract_common::JointTrajectory Gen3MotionPlanner::tcpSpeedLimiter(
    const tesseract_common::JointTrajectory &input_trajectory,
    const double max_speed,
    const std::string tcp)
{
  // Extract objects needed for calculating FK
  tesseract_common::JointTrajectory output_trajectory = input_trajectory;
  tesseract_scene_graph::StateSolver::UPtr state_solver = env_->getStateSolver();

  // Find the adjacent waypoints that require the biggest speed reduction to stay under the max tcp speed
  double strongest_scaling_factor = 1.0;
  for (std::size_t i = 1; i < output_trajectory.size(); i++)
  {
    // Find the previous waypoint position in Cartesian space
    tesseract_scene_graph::SceneState prev_ss =
        state_solver->getState(output_trajectory[i - 1].joint_names, output_trajectory[i - 1].position);
    Eigen::Isometry3d prev_pose = prev_ss.link_transforms[tcp];

    // Find the current waypoint position in Cartesian space
    tesseract_scene_graph::SceneState curr_ss =
        state_solver->getState(output_trajectory[i].joint_names, output_trajectory[i].position);
    Eigen::Isometry3d curr_pose = curr_ss.link_transforms[tcp];

    // Calculate the average TCP velocity between these waypoints
    double dist_traveled = (curr_pose.translation() - prev_pose.translation()).norm();
    double time_to_travel = output_trajectory[i].time - output_trajectory[i - 1].time;
    double original_velocity = dist_traveled / time_to_travel;

    // If the velocity is over the max speed determine the scaling factor and update greatest seen to this point
    if (original_velocity > max_speed)
    {
      double current_needed_scaling_factor = max_speed / original_velocity;
      if (current_needed_scaling_factor < strongest_scaling_factor)
        strongest_scaling_factor = current_needed_scaling_factor;
    }
  }

  // Apply the (updated) strongest scaling factor to all trajectory points to maintain a smooth trajectory
  double total_time = 0;
  for (std::size_t i = 1; i < output_trajectory.size(); i++)
  {
    double original_time_diff = input_trajectory[i].time - input_trajectory[i - 1].time;
    double new_time_diff = original_time_diff / strongest_scaling_factor;
    double new_timestamp = total_time + new_time_diff;
    // Apply new timestamp
    output_trajectory[i].time = new_timestamp;
    // Scale joint velocity by the scaling factor
    output_trajectory[i].velocity = input_trajectory[i].velocity * strongest_scaling_factor;
    // Scale joint acceleartion by the scaling factor squared
    output_trajectory[i].acceleration =
        input_trajectory[i].acceleration * strongest_scaling_factor * strongest_scaling_factor;
    // Update the total running time of the trajectory up to this point
    total_time = new_timestamp;
  }
  // // Display the smoothed joint trajectory
  // for (int i = 0; i < joint_trajectory_.size(); i++)
  // {
  //   std::cout << "Joint Waypoint " << i << std::endl;

  //   // std::cout << joint_trajectory_[i].position << ", \n" <<
  //                std::cout<< joint_trajectory_[i].velocity  << ", \n" << std::endl;//<<
  //                 // joint_trajectory_[i].acceleration  << ", \n" <<
  //                 // joint_trajectory_[i].effort << std::endl;

  // }
  return output_trajectory;
}


tesseract_planning::CompositeInstruction Gen3MotionPlanner::createProgram(
    const tesseract_common::ManipulatorInfo &info,
    const Eigen::Isometry3d &end_pose)
{
  // Get joint names
  std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

  // Create program
  tesseract_planning::CompositeInstruction program(PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED, info);
  
  // Perform a freespace move to the current waypoint
  // since the start move instruction is deprecated
  tesseract_planning::StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));// wp0
  tesseract_planning::StateWaypointPoly current_state_poly{current_state};  // wp0
  tesseract_planning::MoveInstruction start_instrction(current_state_poly, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info);  // start_instruction
  start_instrction.setDescription("Start instruction");

  tesseract_planning::CartesianWaypointPoly wp_1{tesseract_planning::CartesianWaypoint(end_pose)};  //wp1
  tesseract_planning::MoveInstruction move_1(wp_1, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info);
  move_1.setDescription("Target move instruction"); // plan_f0
  // tesseract_planning::CompositeInstruction to_end(PROFILE);
  // to_end.setDescription("Target composite instruction");
  // to_end.appendMoveInstruction(move_1);
  program.appendMoveInstruction(start_instrction);
  program.appendMoveInstruction(move_1);
  // program.push_back(to_end);
  program.print("Program: ");

  // TODO: check if adding a compositeInstruction is necessary
  // tesseract_planning::CompositeInstruction composite;
  return program;
}

#endif // GEN3_MOTION_PLANNER_H