// src/plugin.cpp
#include "behaviortree_cpp/bt_factory.h"
#include "p2p_navigation/custom_nav_to_pose.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<p2p_navigation::CustomNavToPose>("CustomNavToPose");
}