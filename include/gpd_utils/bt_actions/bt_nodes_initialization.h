/*
  @file

  @author victor

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/
#ifndef BT_NODES_INITIALIZATION_H
#define BT_NODES_INITIALIZATION_H

#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <gpd_utils/segment_table_params.h>

namespace pal
{
void initGPDBTNodes(BT::Tree &tree, ros::NodeHandle nh, ros::NodeHandle priv_nh,
                 const pal::PlanarSegmentationParams &segm_params = pal::PlanarSegmentationParams());
}

#endif  // BT_NODES_INITIALIZATION_H
