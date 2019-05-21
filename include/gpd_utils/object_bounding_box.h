/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef OBJECT_BOUNDING_BOX_H
#define OBJECT_BOUNDING_BOX_H

#include <pcl/search/impl/search.hpp>
#include <geometry_msgs/PointStamped.h>

namespace gpd_utils
{
struct BoundingBox
{
public:
  geometry_msgs::PointStamped lu_point_;
  geometry_msgs::PointStamped rb_point_;
  geometry_msgs::PointStamped ctr_point_;
};
}

#endif  // OBJECT_BOUNDING_BOX_H
