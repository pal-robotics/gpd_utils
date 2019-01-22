#include <gpd_utils/states/tabletop_segmentation_state.h>

using namespace pal;

TableTopSegmentationState::TableTopSegmentationState(ros::NodeHandle &nh, ros::NodeHandle &pnh,
                                                     const PlanarSegmentationParams &params,
                                                     const int &num_attempts)
  : nh_(nh)
  , pnh_(pnh)
  , segment_plane_(nh_, pnh_, params)
  , num_attempts_(num_attempts)
  , State({ smach_c::SUCCESS, smach_c::PREEMPTED, smach_c::FAILURE })
{
}

TableTopSegmentationState::~TableTopSegmentationState()
{
}

std::string TableTopSegmentationState::execute(smach_c::UserData &user_data)
{
  bool get_data_successful(false);
  for (int i = 0; i < num_attempts_ && !get_data_successful && !preemptRequested(); i++)
    get_data_successful = segment_plane_.performTableSegmentation();

  if (preemptRequested())
  {
    ROS_WARN_STREAM("State preempted, table segmentation interrupted.");
    return smach_c::PREEMPTED;
  }

  if (!get_data_successful)
  {
    ROS_WARN_STREAM("Unable to perform the table segmentation in " << num_attempts_ << " attempts");
    return smach_c::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tabletop_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::CompressedImagePtr image_org_(new sensor_msgs::CompressedImage);
  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
  double table_height = segment_plane_.getTableHeight();

  segment_plane_.getImage(image_org_);
  segment_plane_.getNonPlaneCloud(nonplane_cloud);
  segment_plane_.getOriginalCloud(original_cloud);
  segment_plane_.getPlaneCloud(plane_cloud);
  segment_plane_.getPlaneCoeff(plane_coeff);
  segment_plane_.getTableTopCloud(tabletop_cloud);


  user_data.addOrUpdateProperty("~table_height", table_height);
  user_data.addOrUpdateProperty("~nonplane_cloud", nonplane_cloud);
  user_data.addOrUpdateProperty("~plane_cloud", plane_cloud);
  user_data.addOrUpdateProperty("~plane_coeff", plane_coeff);
  user_data.addOrUpdateProperty("~tabletop_cloud", tabletop_cloud);
  user_data.addOrUpdateProperty("~image_scene", image_org_);
  user_data.addOrUpdateProperty("~original_cloud", original_cloud);

  return smach_c::SUCCESS;
}
