#ifndef TABLETOP_CLUSTERING_ACTION_H
#define TABLETOP_CLUSTERING_ACTION_H

#include <behaviortree_ros_actions/behaviortree_pal_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <gpd_utils/tabletop_detector_class.h>

namespace pal
{
class TableTopClusteringAction : public BT::SyncActionNode
{
public:
  TableTopClusteringAction(const std::string &name, const BT::NodeConfiguration &config);

  virtual ~TableTopClusteringAction();

  static BT::PortsList providedPorts()
  {
    static const BT::PortsList ports = {
      { BT::InputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("tabletop_cloud",
                                                              "Tabletop pointcloud data"),
        BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
            "object_cloud", "The object's pointcloud data in the cluster") }
    };

    return ports;
  }

  void init(ros::NodeHandle nh);

  virtual BT::NodeStatus tick() override;

protected:
  std::unique_ptr<pal::TableTopDetector<pcl::PointXYZRGB>> TTD_;
};
}


#endif  // TABLETOP_CLUSTERING_ACTION_H
