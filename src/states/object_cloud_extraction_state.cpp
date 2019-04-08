#include <gpd_utils/states/object_cloud_extraction_state.h>

using namespace pal;

ObjectCloudExtractionState::ObjectCloudExtractionState(ros::NodeHandle &nh)
  : nh_(nh)
  , TTD_(nh)
  , obj_info_(nh)
  , State({ smach_c::SUCCESS, smach_c::PREEMPTED, smach_c::FAILURE })
{
}

ObjectCloudExtractionState::~ObjectCloudExtractionState()
{
}

std::string ObjectCloudExtractionState::execute(smach_c::UserData &user_data)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tabletop_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::CompressedImagePtr image_scene(new sensor_msgs::CompressedImage);
  std::string desired_object;

  try
  {
    user_data.getPropertyValue("~tabletop_cloud", tabletop_cloud,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~original_cloud", original_cloud,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~desired_object", desired_object,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~image_scene", image_scene,
                               property_bag::RetrievalHandling::THROW);
  }
  catch (property_bag::PropertyException)
  {
    ROS_ERROR_STREAM("Unable to obtain the point cloud and the image info from the propert_bag");
    return smach_c::FAILURE;
  }

  obj_info_.setPointCloud(original_cloud);
  obj_info_.setImage(image_scene);
  gpd_utils::BoundingBox bbox;
  obj_info_.computeObjectBoundingBox(desired_object, bbox);

  user_data.addOrUpdateProperty("~object_cloud", TTD_.getObjectCloud(tabletop_cloud, bbox));
  return smach_c::SUCCESS;
}
