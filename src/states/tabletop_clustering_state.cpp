#include <gpd_utils/states/tabletop_clustering_state.h>

using namespace pal;

TableTopClusteringState::TableTopClusteringState(ros::NodeHandle &nh)
  : nh_(nh), TTD_(nh), State({ smach_c::SUCCESS, smach_c::PREEMPTED, smach_c::FAILURE })
{
}

TableTopClusteringState::~TableTopClusteringState()
{
}

std::string TableTopClusteringState::execute(smach_c::UserData &user_data)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tabletop_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::CompressedImagePtr image_scene(new sensor_msgs::CompressedImage);

  try
  {
    user_data.getPropertyValue("~tabletop_cloud", tabletop_cloud,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~original_cloud", original_cloud,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~image_scene", image_scene,
                               property_bag::RetrievalHandling::THROW);
  }
  catch (property_bag::PropertyException)
  {
    ROS_ERROR_STREAM("Unable to obtain the point cloud and the image info from the propert_bag");
    return smach_c::FAILURE;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  TTD_.extractOneCluster(tabletop_cloud, clustered_cloud);

  user_data.addOrUpdateProperty("~object_cloud", clustered_cloud);
  return smach_c::SUCCESS;
}
