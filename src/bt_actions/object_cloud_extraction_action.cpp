#include <gpd_utils/bt_actions/object_cloud_extraction_action.h>

namespace pal
{
ObjectCloudExtractionAction::ObjectCloudExtractionAction(const std::string &name,
                                                         const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config)
{
  ros::NodeHandle nh;
  init(nh); 
}

ObjectCloudExtractionAction::~ObjectCloudExtractionAction()
{
}

void ObjectCloudExtractionAction::init(ros::NodeHandle nh)
{
  TTD_.reset(new pal::TableTopDetector<pcl::PointXYZRGB>(nh));
  obj_info_.reset(new pal::ObjectRecognitionInfo(nh, ros::Duration(0.5)));
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

  setOutput("object_cloud", TTD_->getObjectCloud(tabletop_cloud.value(), bbox));
  return BT::NodeStatus::SUCCESS;
}
}
