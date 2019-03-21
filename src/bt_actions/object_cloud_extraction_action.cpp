#include <gpd_utils/bt_actions/object_cloud_extraction_action.h>

namespace pal
{
ObjectCloudExtractionAction::ObjectCloudExtractionAction(const std::string &name,
                                                         const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config)
{
}

ObjectCloudExtractionAction::~ObjectCloudExtractionAction()
{
}

void ObjectCloudExtractionAction::init(const ros::NodeHandle &nh)
{
  nh_ = nh;
  TTD_.reset(new pal::TableTopDetector<pcl::PointXYZRGB>(nh_));
  obj_info_.reset(new pal::ObjectRecognitionInfo(nh_, ros::Duration(5.0)));
}

BT::NodeStatus ObjectCloudExtractionAction::tick()
{
  GET_INPUT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, tabletop_cloud);
  GET_INPUT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, original_cloud);
  GET_INPUT(sensor_msgs::CompressedImagePtr, image_scene);
  GET_INPUT(std::string, desired_object, false);
  if (!desired_object)
  {
    ROS_WARN_STREAM("The desired object information is not found in the blackboard");
    return BT::NodeStatus::FAILURE;
  }

  obj_info_->setPointCloud(*original_cloud);
  obj_info_->setImage(*image_scene);
  gpd_utils::BoundingBox bbox;
  obj_info_->computeObjectBoundingBox(*desired_object, bbox);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  TTD_->getObjectCloud(tabletop_cloud.value(), bbox, object_cloud);
  setOutput("object_cloud", object_cloud);
  return BT::NodeStatus::SUCCESS;
}
}
