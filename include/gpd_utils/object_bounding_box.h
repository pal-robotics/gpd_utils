#ifndef OBJECT_BOUNDING_BOX_H
#define OBJECT_BOUNDING_BOX_H

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
