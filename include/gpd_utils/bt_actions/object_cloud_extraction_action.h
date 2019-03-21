#ifndef OBJECT_CLOUD_EXTRACTION_ACTION_H
#define OBJECT_CLOUD_EXTRACTION_ACTION_H

#include <behaviortree_ros_actions/behaviortree_pal_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <gpd_utils/tabletop_detector_class.h>
#include <gpd_utils/object_recognition_info.h>
#include <gpd_utils/object_bounding_box.h>

namespace pal
{
class ObjectCloudExtractionAction : public BT::SyncActionNode
{
public:
  ObjectCloudExtractionAction(const std::string &name, const BT::NodeConfiguration &config);

  virtual ~ObjectCloudExtractionAction();

  static BT::PortsList providedPorts()
  {
    static const BT::PortsList ports = {
      { BT::InputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("original_cloud",
                                                              "Original Pointcloud Data"),
        BT::InputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("tabletop_cloud",
                                                              "Tabletop pointcloud data"),
        BT::InputPort<sensor_msgs::CompressedImagePtr>(
            "image_scene", "Image scene corresponding to the point cloud"),
        BT::InputPort<std::string>("desired_object", "Desired object to grasp"),
        BT::OutputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
            "object_cloud", "The desired object's point cloud") }
    };

    return ports;
  }

  virtual BT::NodeStatus tick() override;

  void init(const ros::NodeHandle &nh);

protected:
  ros::NodeHandle nh_;
  std::unique_ptr<pal::TableTopDetector<pcl::PointXYZRGB>> TTD_;
  std::unique_ptr<ObjectRecognitionInfo> obj_info_;
};
}

#endif  // OBJECT_CLOUD_EXTRACTION_ACTION_H
