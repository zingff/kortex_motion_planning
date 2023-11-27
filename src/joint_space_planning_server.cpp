#include <kortex_motion_planning/joint_space_planner_profiles.hpp>
#include <ros/ros.h>
#include <ros/package.h>
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
#include <kortex_motion_planning/GenerateKortexJointMotionPlan.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "GEN3";
static const std::string PROFILE2 = "GEN3_FREESPACE";
static const std::string PLANNING_SERVICE = "/motion_planning_server";
static const std::string MONITOR_NAMESPACE = "gen3_environment";
static const std::string TASK_PIPELINE = "Gen3Pipeline";
static const std::string STATE_FEEDBACK_TOPIC = "/base_feedback";

static const double MAX_TCP_SPEED = 0.2; // m/s

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

static const int DOF = 7;

static const std::string MANIPULATOR_GROUP_NAME = "manipulator";
// static const std::string WORKING_FRAME_NAME = "link_0";
// static const std::string TCP_FRAME_NAME = "end_effector_link";
static const std::string WORKING_FRAME_NAME = "base_link";
static const std::string TCP_FRAME_NAME = "tool_frame";

static const std::string TASK_COMPOSER_PLUGIN_PKG_NAME = "kortex_motion_planning";
static const std::string TASK_COMPOSER_PLUGIN_SUB_PATH = "/config/tesseract/task_composer_plugins.yaml";

// static const std::string URDF_FILE_PATH = "kortex_motion_planning/config/robot/gen3_robotiq_2f_85_tesseract.urdf";
// static const std::string SRDF_FILE_PATH = "kortex_motion_planning/config/robot/gen3_robotiq_2f_85_tesseract.srdf";

static const std::string URDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.urdf";
static const std::string SRDF_FILE_PATH = "kortex_description/robots/urdf/gen3_robotiq_2f_85_tesseract.srdf";

const double JOINT_1_LIMIT = 2.41;
const double JOINT_3_LIMIT = 2.66;
const double JOINT_5_LIMIT = 2.23;

// TODO: modify the server within the class
// suanle youdiankongbu

class PlanningServer
{
  public:
  PlanningServer(ros::NodeHandle n) 
  : nh_(n)
  , env_(std::make_shared<tesseract_environment::Environment>())
  {
    // Load parameter from param server
    ROS_INFO("Ready to load parameters!");
    n.param("plotting", plotting_, true);
    n.param("rviz", rviz_, true);
    n.param("ifopt", ifopt_, false);
    n.param("debug", debug_, false);
    n.param("verbose", verbose_, false);
    n.param("method_id", method_id_, 1);
    n.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string_);
    n.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string_);

    // Create robot model
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    ROS_INFO("Ready to create robot model!");
    auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    tesseract_common::fs::path urdf_path = locator->locateResource("package://" + URDF_FILE_PATH)->getFilePath();
    tesseract_common::fs::path srdf_path = locator->locateResource("package://" + SRDF_FILE_PATH)->getFilePath();

    if (!env_->init(urdf_xml_string_, srdf_xml_string_, locator))
    {
      ROS_ERROR("Failed to initialize environment!");
    }
    ROS_INFO("Environment initialized!");

    urdf_model_ = urdf::parseURDF(urdf_xml_string_);
    ROS_INFO("URDF parsed!");
    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model_->initString(*urdf_model_, srdf_xml_string_);
    ROS_INFO("SRDF initialized!");
    assert(urdf_model_ != nullptr && srdf_model_ != nullptr);
    robot_model_.reset(new moveit::core::RobotModel(urdf_model_, srdf_model_));
    ROS_INFO("Robot model loaded!");

    manipulator_info_ = tesseract_common::ManipulatorInfo(MANIPULATOR_GROUP_NAME, WORKING_FRAME_NAME, TCP_FRAME_NAME);
    std::vector<std::string> joint_names_ = env_->getJointGroup(manipulator_info_.manipulator)->getJointNames();
    ROS_INFO("Joint names got!");

    getCurrentPosition();
    env_->setState(joint_names_, current_joint_position_);

    // Create environment monitor
    monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env_, MONITOR_NAMESPACE);
    monitor_->setEnvironmentPublishingFrequency(40);
    monitor_->startPublishingEnvironment();
    monitor_->startStateMonitor("/joint_states", true);

    // Create plotter
    if(plotting_)
    {
      // plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getRootLinkName());
      plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());  // equivalent
      ROS_INFO("Monitor created!");

      // TODO: here to define the original exampel class

      profile_dictionary_ = std::make_shared<tesseract_planning::ProfileDictionary>();

      // Add custom profiles
      profile_dictionary_->addProfile<tesseract_planning::SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, PROFILE,
        createSimplePlannerProfile());
      ROS_INFO("Simple profile added!");

      // profile_dictionary_->addProfile<tesseract_planning::OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, PROFILE, 
      //   createOMPLProfile());
      // ROS_INFO("OMPL profile added!");

      profile_dictionary_->addProfile<tesseract_planning::TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE, 
        createTrajOptCompositeProfile());
      ROS_INFO("TrajOpt composite profile added!");

      profile_dictionary_->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptPlanProfile());
      ROS_INFO("TrajOpt plan profile added!");

      // profile_dictionary_->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptToolZFreePlanProfile());
      // ROS_INFO("TrajOpt plan profile (z axis rotation-free) added!");

      // profile_dictionary_->addProfile<tesseract_planning::DescartesPlanProfile<float>>(
      //   DESCARTES_DEFAULT_NAMESPACE, PROFILE, 
      //   createDescartesPlanProfile<float>());
      // ROS_INFO("Descartes profile added!");

      // profile_dictionary_->addProfile<tesseract_planning::MinLengthProfile>(
      //   MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE,
      //   std::make_shared<tesseract_planning::MinLengthProfile>(5));
      // ROS_INFO("Min length profile added!");

      // profile_dictionary_->addProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
      //   ISP_DEFAULT_NAMESPACE, PROFILE,
      //   std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>());
      // ROS_INFO("ISP profile added!");

      ROS_INFO("Plan profile created");
    }
  }

  // bool plan(
  //   kortex_motion_planning::GenerateKortexMotionPlan::Request &req,
  //   kortex_motion_planning::GenerateKortexMotionPlan::Response &res
  // )
  bool plan(
    kortex_motion_planning::GenerateKortexJointMotionPlan::Request &req,
    kortex_motion_planning::GenerateKortexJointMotionPlan::Response &res
  )
  {
    try
    {
      ROS_INFO("Received kortex motion planning request!");

      // TODO: load robot model
      // Create manipulator info
      tesseract_common::ManipulatorInfo manipulator_info(MANIPULATOR_GROUP_NAME, WORKING_FRAME_NAME, TCP_FRAME_NAME);
      ROS_INFO("Manipulator info created!");

      // Initialize start configuration with current joint positions
      joint_names_ = env_->getJointGroup(manipulator_info.manipulator)->getJointNames();
      ROS_INFO("Joint names got!");
      auto stateFeedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(STATE_FEEDBACK_TOPIC);
      Eigen::VectorXd start_configuration(DOF);
      std::cout << "hhhhh" << std::endl;
      for (int i = 0; i < DOF; i++)
      {
        start_configuration(i) = stateFeedback->actuators[i].position/(180.0/M_PI);
        std::cout << start_configuration(i) << std::endl;
      }
      start_configuration = adjustJointPositions(start_configuration);

      std::cout << "1" << std::endl;
      // TODO: to modify
      // Initialize target configuration
      Eigen::VectorXd target_joint_positions(DOF);
      std::cout << "2" << std::endl;
      for (size_t i = 0; i < DOF; i++)
      {
              std::cout << "3" << std::endl;
        target_joint_positions(i) = req.target_positions.joint_positions[i];
      }   
            std::cout << "4" << std::endl;
      target_joint_positions = adjustJointPositions(target_joint_positions);

      // Set initial state in the tesseract environment
            std::cout << "5" << std::endl;
      env_->setState(joint_names_, start_configuration);
      std::cout << "6" << std::endl;
      // Set up composite instruction and environment
      tesseract_planning::CompositeInstruction program = createProgram(manipulator_info, target_joint_positions);
      std::cout << "7" << std::endl;
      // Set up task composer problem
      std::string config_path = ros::package::getPath(TASK_COMPOSER_PLUGIN_PKG_NAME);
            std::cout << "8" << std::endl;
      if (config_path.empty())
      {
        ROS_ERROR("Failed to get path of config file, please check!");
      }
            std::cout << "9" << std::endl;
      config_path += TASK_COMPOSER_PLUGIN_SUB_PATH;
      tesseract_planning::TaskComposerPluginFactory factory(YAML::LoadFile(config_path));
      ROS_INFO("Task composer plugin factory created!");

      // Create executor      
      auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

      // Create task
      // TODO: Gen3Pipeline 1.0 done copy of CartesianPipeline
      // TODO: Gen3Pipeline 2.0 done simple->TrajOpt
      tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(TASK_PIPELINE);
      const std::string input_key = task->getInputKeys().front();
      const std::string output_key = task->getOutputKeys().front();
      ROS_INFO("Task composer node created!");

      // Save dot graph
      std::ofstream task_composer_data;
      task_composer_data.open(tesseract_common::getTempPath() + TASK_PIPELINE + ".dot");  // /temp
      task->dump(task_composer_data);
      ROS_INFO("Dot graph saved!");

      // Create task input data
      tesseract_planning::TaskComposerDataStorage input_data;
      input_data.setData(input_key, program);

      // Create task composer problem
      tesseract_planning::TaskComposerProblem problem(env_, input_data);

      // Update log level for debugging
      auto log_level = console_bridge::getLogLevel();
      std::cout << "Log level: " << log_level << std::endl;
      if (verbose_)
      {
        // verbose_ = False currently
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

        // Create dump dotgraphs of each task for reference
        std::ofstream cartesian_pipeline_output_data;
        cartesian_pipeline_output_data.open(tesseract_common::getTempPath() + "Gen3CartesianPipeline.dot");
        factory.createTaskComposerNode("Gen3CartesianPipeline")->dump(cartesian_pipeline_output_data);

        std::ofstream freespace_pipeline_output_data;
        freespace_pipeline_output_data.open(tesseract_common::getTempPath() + "Gen3FreespacePipeline.dot");
        factory.createTaskComposerNode("Gen3FreespacePipeline")->dump(freespace_pipeline_output_data);
        // TODO: check if other pipeline output data is needed
      }

      // Solve process plan
      ROS_INFO("Begin to solve the program!");
      tesseract_common::Timer stopwatch;  // get solving time
      stopwatch.start();
      tesseract_planning::TaskComposerInput input(problem, profile_dictionary_);  // passed as an input to each process in the decision tree
      tesseract_planning::TaskComposerFuture::UPtr executor_future = executor->run(*task, input);
      executor_future->wait();
      stopwatch.stop();
      ROS_INFO("Planning time: %f seconds", stopwatch.elapsedSeconds());

      // Reset the log level
      console_bridge::setLogLevel(log_level);

      // Get motion planning result
      tesseract_planning::CompositeInstruction program_result = input.data_storage.getData(output_key).as<tesseract_planning::CompositeInstruction>();
      tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(program_result, *env_);

      // Check for successful plan
      if (!input.isSuccessful())
      {
        // throw std::runtime_error("Failed to create motion plan!");
        ROS_ERROR("Failed to create motion plan!");
      }
      else
      {
        ROS_INFO("Succeeded to create motion plan!");
      }

      // // Display waypoints
      // // TODO: to use CatesianWaypoint.print()
      // ROS_INFO("Waypoint number: %d", toolpath.data()->size());
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
      res.motion_plan = tesseract_rosutils::toMsg(tcp_velocity_scaled_joint_trajectory, env_->getState());
      res.message = "Motion planning succeeded!";
      res.success = true;
      }
      catch (const std::exception& ex)
      {
        res.message = ex.what();
        res.success = false;
        return false;
      }
      return res.success;
  }

  private:

  // TODO: only starting and target waypoints are given
  tesseract_planning::CompositeInstruction createProgram(
    const tesseract_common::ManipulatorInfo& info, 
    // const Eigen::Isometry3d& end_pose,  // 2joint
    const Eigen::VectorXd& target_positions
  )
  {
    // Get joint names
    std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    // Create program
    tesseract_planning::CompositeInstruction program(PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED, info);
    
    // Perform a freespace move to the current waypoint
    // since the start move instruction is deprecated
    tesseract_planning::StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));// wp0
    tesseract_planning::StateWaypointPoly current_state_poly{current_state};  // wp0
    tesseract_planning::MoveInstruction start_instrction(current_state_poly, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info);  // start_instruction
    start_instrction.setDescription("Start instruction");

    tesseract_planning::StateWaypoint target_state(joint_names, target_positions);
    tesseract_planning::StateWaypointPoly target_state_poly{target_state};  // wp1

    // tesseract_planning::CartesianWaypointPoly wp_1{tesseract_planning::CartesianWaypoint(end_pose)};  //wp1

    tesseract_planning::MoveInstruction move_1(target_state_poly, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info);
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

  tesseract_common::JointTrajectory tcpSpeedLimiter(
    const tesseract_common::JointTrajectory& input_trajectory,
    const double max_speed, 
    const std::string tcp
  )
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

  void getCurrentPosition()
  {
    current_joint_position_ = Eigen::VectorXd(DOF);
    auto stateFeedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>(STATE_FEEDBACK_TOPIC);
    for (int i = 0; i < DOF; i++)
    {
      current_joint_position_(i) = stateFeedback->actuators[i].position / (180 / M_PI);
    }
  }

  Eigen::VectorXd adjustJointPositions(const Eigen::VectorXd& original_positions) {
      Eigen::VectorXd adjusted_positions = original_positions;
      std::cout << "gggg" << std::endl;

      for (int i = 0; i < DOF; i++) {
          if (i == 1 || i == 3 || i == 5) {
              double joint_position = adjusted_positions(i);
              std::cout << "kkk" <<  std::endl;

              if ((i == 1 && std::abs(joint_position) >= JOINT_1_LIMIT) || 
                  (i == 3 && std::abs(joint_position) >= JOINT_3_LIMIT) ||
                  (i == 5 && std::abs(joint_position) >= JOINT_5_LIMIT)) {
                  
                  if (joint_position < 0) {
                      joint_position += 2 * M_PI;
                  } else {
                      joint_position -= 2 * M_PI;
                  }

                  adjusted_positions(i) = joint_position;
              }
          }
      }
      for (size_t i = 0; i < DOF; i++)
      {
        std::cout << adjusted_positions(i) << std::endl;
      }
      

      return adjusted_positions;
  }

  ros::NodeHandle nh_;
  bool plotting_ = true;
  bool rviz_ = true;
  bool ifopt_ = false;
  bool verbose_ = false;  // log details
  bool debug_ = false;
  int method_id_;

  tesseract_common::JointTrajectory joint_trajectory_;

  std::string urdf_xml_string_, srdf_xml_string_;
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

};

int main(int argc, char** argv)
{
  ROS_INFO("Initializing planning server!");
  // TODO: to use const string
  ros::init(argc, argv, "gen3_motion_planning_node");
  ros::NodeHandle n;

  PlanningServer gen3_motion_planner(n);
  ros::ServiceServer gen3_motion_planning_server = n.advertiseService(PLANNING_SERVICE, &PlanningServer::plan, &gen3_motion_planner);

  ROS_INFO("Ready to provide trajectory for Kinova Gen3!");

  ros::spin();
  return 0;
}


// TODO: modify to the following: 
// #include <kortex_motion_planning/planning_server.h> 
// #include <kortex_motion_planning/gen3_motion_planner.h> 
// #include <ros/ros.h>
// #include <ros/package.h>
// int main(int argc, char** argv)
// {
//     ROS_INFO("Initializing planning server!");
//     // TODO: to use const string
//     ros::init(argc, argv, "gen3_motion_planning_node");
//     ros::NodeHandle n;

//     // PlanningServer gen3_motion_planner(n);
//     // ros::ServiceServer gen3_motion_planning_server = n.advertiseService(PLANNING_SERVICE, &PlanningServer::plan, &gen3_motion_planner);

//     Gen3MotionPlanner gen3_motion_planner;
//     // gen3_motion_planner.createGen3MotionPlan()

//     ROS_INFO("Ready to provide trajectory for Kinova Gen3!");

//     ros::spin();
//     return 0;
// }