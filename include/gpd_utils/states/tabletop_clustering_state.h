#ifndef TABLETOP_CLUSTERING_STATE_H
#define TABLETOP_CLUSTERING_STATE_H

#include <ros/ros.h>
#include <smach_c/state.h>
#include <sensor_msgs/CompressedImage.h>
#include <gpd_utils/tabletop_detector_class.h>

namespace pal
{
/**
 * @brief The TableTopClusteringState class - helps to get the cluster in the pointcloud
 * and use the info for grasping
 */
class TableTopClusteringState : public smach_c::State
{
public:
  // Outcomes are: smach_c::SUCCESS when the TableTopClusteringState is successful
  //               smach_c::FAILURE in the case of failure
  //               smach_c::PREEMPTED in the case of preemption

  TableTopClusteringState(ros::NodeHandle &nh);

  virtual ~TableTopClusteringState();

  virtual std::string execute(smach_c::UserData &user_data) override;

protected:
  ros::NodeHandle nh_;
  pal::TableTopDetector<pcl::PointXYZRGB> TTD_;
};
}

#endif  // TABLETOP_CLUSTERING_STATE_H
