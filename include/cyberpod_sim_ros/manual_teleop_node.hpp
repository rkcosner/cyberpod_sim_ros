#ifndef MANUAL_TELEOP_NODE_H
#define MANUAL_TELEOP_NODE_H

#include "ros/ros.h"
#include "cyberpod_sim_ros/input.h"
#include "cyberpod_sim_ros/state.h"
#include "cyberpod_sim_ros/cmd.h"
#include "cyberpod_sim_ros/common.hpp"
#include "cyberpod_sim_ros/keyboard.hpp"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <stdlib.h>

enum class STATUS : uint8_t
{
  FAILURE = 0,
  RUNNING = 1
};

std::map<char, Eigen::Vector2d> moveBindings
{
  {'w', { 1., 0.}},
  {'s', {-1., 0.}},
  {'a', { 0., 1.}},
  {'d', { 0.,-1.}},
};

#endif
