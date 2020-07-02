#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "ros/ros.h"
#include "cyberpod_sim_ros/common.hpp"
#include "cyberpod_sim_ros/input.h"
#include "cyberpod_sim_ros/state.h"
#include "cyberpod_sim_ros/cmd.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/Image.h"
#include "cyberpod_sim_ros/learning_data.h"

enum class STATUS : uint8_t
{
	FAILURE = 0,
	RUNNING = 1
};

#endif
