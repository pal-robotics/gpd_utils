/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef OBJECT_CLOUD_EXTRACTION_STATE_H
#define OBJECT_CLOUD_EXTRACTION_STATE_H

#include <ros/ros.h>
#include <smach_c/state.h>
#include <sensor_msgs/CompressedImage.h>
#include <gpd_utils/tabletop_detector_class.h>
#include <gpd_utils/object_recognition_info.h>
#include <gpd_utils/object_bounding_box.h>

namespace pal
{
/**
 * @brief The ObjectCloudExtractionState class - helps to get the cluster of an individual
 * recognisable object in the pointcloud and use the info for grasping
 */
class ObjectCloudExtractionState : public smach_c::State
{
public:
  // Outcomes are: smach_c::SUCCESS when the ObjectCloudExtractionState is successful
  //               smach_c::FAILURE in the case of failure
  //               smach_c::PREEMPTED in the case of preemption

  ObjectCloudExtractionState(ros::NodeHandle &nh);

  virtual ~ObjectCloudExtractionState();

  virtual std::string execute(smach_c::UserData &user_data) override;

protected:
  ros::NodeHandle nh_;
  pal::TableTopDetector<pcl::PointXYZRGB> TTD_;
  ObjectRecognitionInfo obj_info_;
};
}

#endif  // OBJECT_CLOUD_EXTRACTION_STATE_H
