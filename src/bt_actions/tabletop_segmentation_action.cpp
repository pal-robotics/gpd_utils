#include <gpd_utils/bt_actions/tabletop_segmentation_action.h>
#include <ros/ros.h>

namespace pal
{
TableTopSegmentationAction::TableTopSegmentationAction(const std::string &name,
                                                       const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config)
{
}

TableTopSegmentationAction::~TableTopSegmentationAction()
{
}

void TableTopSegmentationAction::init(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
                                      const PlanarSegmentationParams &params)
{
  nh_ = nh;
  pnh_ = pnh;
  segment_plane_.reset(new PlanarSegmentation<pcl::PointXYZRGB>(nh_, pnh_, params));
}

BT::NodeStatus TableTopSegmentationAction::tick()
{
  bool get_data_successful(false);
  get_data_successful = segment_plane_->performTableSegmentation();

  if (!get_data_successful)
  {
    ROS_WARN_STREAM("Unable to perform the table segmentation");
    return BT::NodeStatus::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tabletop_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::CompressedImagePtr image_org(new sensor_msgs::CompressedImage);
  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
  double table_height = segment_plane_->getTableHeight();
  // added 1 cm offset to the actual table height to prevent close grasps
  table_height = table_height + 0.01;

  segment_plane_->getImage(image_org);
  segment_plane_->getNonPlaneCloud(nonplane_cloud);
  segment_plane_->getOriginalCloud(original_cloud);
  segment_plane_->getOriginalTransformedCloud(original_cloud_transformed);
  segment_plane_->getPlaneCloud(plane_cloud);
  segment_plane_->getPlaneCoeff(plane_coeff);
  segment_plane_->getTableTopCloud(tabletop_cloud);


  setOutput("table_height", table_height);
  setOutput("nonplane_cloud", nonplane_cloud);
  setOutput("plane_cloud", plane_cloud);
  setOutput("plane_coeff", plane_coeff);
  setOutput("tabletop_cloud", tabletop_cloud);
  setOutput("image_scene", image_org);
  setOutput("original_cloud", original_cloud);
  setOutput("original_cloud_transformed", original_cloud_transformed);

  return BT::NodeStatus::SUCCESS;
}
}
