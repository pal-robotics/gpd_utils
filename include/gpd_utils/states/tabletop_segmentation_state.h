/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef TABLETOP_SEGMENTATION_STATE_H
#define TABLETOP_SEGMENTATION_STATE_H

#include <ariles/adapters_all.h>
#include <ros/ros.h>
#include <smach_c/state.h>
#include <gpd_utils/segment_table.h>

namespace pal
{
/**
 * @brief The TableTopSegmentationState class - This state helps to obtain the various
 * pointcloud information along with the image information
 */
class TableTopSegmentationState : public smach_c::State
{
public:
  // Outcomes are: smach_c::SUCCESS when the TableTopSegmentationState is successful
  //               smach_c::FAILURE in the case of failure
  //               smach_c::PREEMPTED in the case of preemption

  TableTopSegmentationState(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
                            const PlanarSegmentationParams &params, int num_attempts = 1);

  virtual ~TableTopSegmentationState();

  virtual std::string execute(smach_c::UserData &user_data) override;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  int num_attempts_;
  PlanarSegmentation<pcl::PointXYZRGB> segment_plane_;
};
}

#endif  // TABLETOP_SEGMENTATION_STATE_H
