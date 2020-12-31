cmake_minimum_required(VERSION 2.8.3)
project(cyberpod_sim_ros)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	std_msgs
	sensor_msgs
	geometry_msgs
	visualization_msgs
	nav_msgs
	tf
	pcl_conversions
	pcl_ros
	)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED CONFIG)
find_package(asif++ REQUIRED CONFIG)
find_package(Threads REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  input.msg
  state.msg
  statehardware.msg
  ctrlinfo.msg
  cmd.msg
  filterInfo.msg
  sensor.msg
  learning_data.msg
  output_data.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ui.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
	             message_generation
	             std_msgs
	             sensor_msgs
	             geometry_msgs
	             visualization_msgs
	             nav_msgs
	             tf
	             pcl_conversions
	             pcl_ros)

###########
## Build ##
###########
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Add Python Node
catkin_install_python(PROGRAMS src/src_python/ecos_filter_node.py src/src_python/data_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(integrator_node src/integrator_node.cpp)
add_executable(controller_node src/controller_node.cpp)
add_executable(manual_teleop_node src/manual_teleop_node.cpp src/keyboard.cpp)
add_executable(converter_node src/converter_node.cpp)
add_executable(sensor_node src/sensor_node.cpp)
add_executable(image_processor_node src/image_processor_node.cpp)
add_executable(grid_state_space_node src/grid_state_space_node.cpp)


# Ensure messages built first
add_dependencies(integrator_node cyberpod_sim_ros_generate_messages_cpp)
add_dependencies(controller_node cyberpod_sim_ros_generate_messages_cpp)
add_dependencies(manual_teleop_node cyberpod_sim_ros_generate_messages_cpp)
add_dependencies(converter_node cyberpod_sim_ros_generate_messages_cpp)
add_dependencies(sensor_node cyberpod_sim_ros_generate_messages_cpp)
add_dependencies(image_processor_node cyberpod_sim_ros_generate_messages_cpp)
add_dependencies(grid_state_space_node cyberpod_sim_ros_generate_messages_cpp)

target_link_libraries(integrator_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)

target_link_libraries(controller_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)


target_link_libraries(manual_teleop_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
                      Threads::Threads
)

target_link_libraries(converter_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)

target_link_libraries(sensor_node
                      ${catkin_LIBRARIES}
)


target_link_libraries(image_processor_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)

target_link_libraries(grid_state_space_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)