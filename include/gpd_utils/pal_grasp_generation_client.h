#ifndef PAL_GRASP_GENERATION_CLIENT_H
#define PAL_GRASP_GENERATION_CLIENT_H

// system
#include <algorithm>
#include <vector>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/client/simple_action_client.h>

// this project (messages)
#include <gpd_utils/GraspCandidatesGenerationAction.h>

namespace pal
{
/** PalGraspGenerationClient class
 *
 * \brief A ROS node that can acts as client for the generation grasp poses in a point
 * cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class PalGraspGenerationClient
{
public:
  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  PalGraspGenerationClient(ros::NodeHandle& node);

  /**
   * \brief Destructor.
  */
  ~PalGraspGenerationClient()
  {
  }

  /**
   * \brief Run the ROS node. Loops while waiting for incoming ROS messages.
  */
  void run();

private:
  /**
   * \brief Callback function for the ROS topic that contains the input point cloud.
   * \param msg the incoming ROS message
  */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  bool has_cloud_;
  sensor_msgs::PointCloud2 cloud_data_;
  ros::Subscriber cloud_sub_;  /// ROS subscriber for point cloud messages

  ros::NodeHandle nh_;

  actionlib::SimpleActionClient<gpd_utils::GraspCandidatesGenerationAction> grasp_generation_client_;
};
}

#endif  // PAL_GRASP_GENERATION_CLIENT_H
