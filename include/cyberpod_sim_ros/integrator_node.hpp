#ifndef INTEGRATOR_NODE_H
#define INTEGRATOR_NODE_H



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "cyberpod_sim_ros/input.h"
#include "cyberpod_sim_ros/state.h"
#include "cyberpod_sim_ros/ui.h"
#include "cyberpod_sim_ros/common.hpp"
#include "cyberpod_sim_ros/dynamics.hpp"

#include <tf/transform_broadcaster.h>

#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;
typedef std::vector<double> state_t;
typedef runge_kutta4<state_t> stepper_t;

enum class STATUS : uint8_t
{
	FAILURE = 0,
	STOPPED = 1,
	RUNNING = 2
};

enum class CMD : uint8_t
{
	STOP = 0,
	START = 1,
	PAUSE = 2,
	REPOSITION = 3,
	RESET = 4
};

#endif
