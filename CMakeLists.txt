cmake_minimum_required(VERSION 3.5)
project(kortex_motion_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT PINOCCHIO_LIB_DIRS)
  set(PINOCCHIO_LIB_DIRS 
    "${PROJECT_SOURCE_DIR}/../../devel/lib"
  )
endif()

SET(KORTEX_CONFIG_DIR 
  "${PROJECT_SOURCE_DIR}/config"
)
add_definitions(-DKORTEX_CONFIG_DIR="${KORTEX_CONFIG_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  moveit_core
  # moveit_ros_planning
  moveit_ros_planning_interface
  std_srvs
  tesseract_monitoring
  tesseract_rosutils
  message_generation
)

find_package(pinocchio REQUIRED)
find_package(descartes_light REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(Qt5 REQUIRED Widgets)
find_package(QCustomPlot REQUIRED)

# find_package(LAPACK REQUIRED) # Requried for ikfast

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../ros_kortex/kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../ros_kortex/kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../ros_kortex/kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../ros_kortex/kortex_api/include/common)
include_directories(${QCustomPlot_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_message_files(
  FILES
  JointPositions.msg
)
add_service_files(
  FILES
  GenerateKortexMotionPlan.srv
  ExecuteMotionPlan.srv
  GenerateKortexJointMotionPlan.srv
  KortexSimpleJmpe.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
)

# Gripper command
add_executable(gripper_command 
  src/utilities/gripper_command.cpp
)
add_dependencies(gripper_command 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gripper_command 
  ${catkin_LIBRARIES}
)

# Planning server
add_executable(${PROJECT_NAME}_node 
  src/planning_server.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(${PROJECT_NAME}_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)

# Joint space planning server
add_executable(kortex_joint_motion_planning_node 
  src/joint_space_planning_server.cpp
)
target_link_libraries(
  kortex_joint_motion_planning_node
  ${catkin_LIBRARIES}
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
)

target_include_directories(kortex_joint_motion_planning_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(kortex_joint_motion_planning_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)

# Motion widget for joint widget
add_executable(kortex_joint_motion_widget_node 
  src/joint_main.cpp
)
target_link_libraries(
  kortex_joint_motion_widget_node
  ${catkin_LIBRARIES}
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
)
target_include_directories(kortex_joint_motion_widget_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(kortex_joint_motion_widget_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)


# Planning server new
add_executable(${PROJECT_NAME}_new_node 
  src/planning_server_new.cpp)
target_link_libraries(
  ${PROJECT_NAME}_new_node
  ${catkin_LIBRARIES}  # add this line to use ros properly
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
)

target_include_directories(${PROJECT_NAME}_new_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)  # to use headers from tesseract packages
target_include_directories(${PROJECT_NAME}_new_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)  # to use headers from tesseract packages

# Motion executor
add_executable(kortex_motion_execution_node src/motion_execution.cpp)
target_link_libraries(
  kortex_motion_execution_node
  ${catkin_LIBRARIES}  # add this line to use ros properly
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp)

target_include_directories(kortex_motion_execution_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)  # to use headers from tesseract packages
target_include_directories(kortex_motion_execution_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)  # to use headers from tesseract packages

# Motion widget
add_executable(kortex_motion_widget_node 
  src/main.cpp
)
target_link_libraries(
  kortex_motion_widget_node
  ${catkin_LIBRARIES}
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
)
target_include_directories(kortex_motion_widget_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(kortex_motion_widget_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)

message("Pinocchio include dirs: ${pinocchio_INCLUDE_DIRS}")
# /home/zing/mealAssistiveRobot/sla_ws/devel/include
message("Pinocchio library dirs: ${pinocchio_LIBRARY_DIRS}")
# none
    
# collision detection
link_directories(${PINOCCHIO_LIB_DIRS}) # libpinocchio
add_executable(collision_detection 
  src/collision_detection/collision_detection.cpp
)
target_link_libraries(
  collision_detection
  ${catkin_LIBRARIES}  # add this line to use ros properly
  pinocchio
  Qt5::Widgets
  ${QCustomPlot_LIBRARIES}
)

# joint torque visualization
add_executable(joint_torque_visualization 
  src/utilities/joint_torque_visualization.cpp
)
target_link_libraries(
  joint_torque_visualization
  ${catkin_LIBRARIES}  # add this line to use ros properly
  Qt5::Widgets
  ${QCustomPlot_LIBRARIES}
)

# collision detection visualization
link_directories(${PINOCCHIO_LIB_DIRS}) # libpinocchio
add_executable(collision_detection_visualization 
  src/collision_detection/collision_detection_visualization.cpp
)
target_link_libraries(
  collision_detection_visualization
  ${catkin_LIBRARIES}  # add this line to use ros properly
  Qt5::Widgets
  ${QCustomPlot_LIBRARIES}
  pinocchio
)

# simple jmpe
add_executable(simple_jmpe
	src/simple_jmpe.cpp
)
target_link_libraries(simple_jmpe 
  ${catkin_LIBRARIES}
)
add_dependencies(simple_jmpe 
  ${catkin_EXPORTED_TARGETS}
)



#####################################
########## FOR UNCLEAR USE ##########
#####################################

# Plugin Library
add_library(${PROJECT_NAME}_plugins SHARED 
  src/motoman_hc10_ikfast_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC 
  ${LAPACK_LIBRARIES} 
  tesseract::tesseract_common
  tesseract::tesseract_kinematics_ikfast
)
target_include_directories(${PROJECT_NAME}_plugins SYSTEM PUBLIC 
  ${LAPACK_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)

# Install the executable
install(TARGETS ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME})
# Install the library
install(TARGETS ${PROJECT_NAME}_plugins EXPORT 
  ${PROJECT_NAME}-targets DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

add_library(${PROJECT_NAME} 
  src/gen3_motion_planner.cpp
  include/kortex_motion_planning/planner_profiles.hpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)