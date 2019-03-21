#ifndef TABLETOP_SEGMENTATION_ACTION_H
#define TABLETOP_SEGMENTATION_ACTION_H

#include <behaviortree_ros_actions/behaviortree_pal_utils.h>
#include <ariles/adapters_all.h>
#include <ros/ros.h>
#include <gpd_utils/segment_table.h>

namespace pal
{
class TableTopSegmentationAction : public BT::SyncActionNode
{
public:
  TableTopSegmentationAction(const std::string &name, const BT::NodeConfiguration &config);

  virtual ~TableTopSegmentationAction();

  static BT::PortsList providedPorts()
  {
    static const BT::PortsList ports = {
      { BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("original_cloud",
                                                               "Original Pointcloud Data"),
        BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
            "original_cloud_transformed", "Transformed Original Pointcloud Data"),
        BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("tabletop_cloud",
                                                               "Tabletop pointcloud data"),
        BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("nonplane_cloud",
                                                               "Non-planar pointcloud data"),
        BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("plane_cloud", "Planar pointcloud data"),
        BT::OutputPort<pcl::ModelCoefficients::Ptr>("plane_cloud", "Detected Plane coeff"),
        BT::OutputPort<double>("table_height", "Table height in meters"),
        BT::OutputPort<sensor_msgs::CompressedImagePtr>(
            "image_scene", "Image scene corresponding to the point cloud") }
    };

    return ports;
  }

  virtual BT::NodeStatus tick() override;

  void init(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
            const PlanarSegmentationParams &params);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<PlanarSegmentation<pcl::PointXYZRGB>> segment_plane_;
};
}

#endif  // TABLETOP_SEGMENTATION_ACTION_H
