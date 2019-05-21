/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
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
