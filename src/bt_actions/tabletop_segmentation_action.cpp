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
  bool get_data_successful = segment_plane_->performTableSegmentation();

  if (!get_data_successful)
  {
    ROS_WARN_STREAM("Unable to perform the table segmentation");
    return BT::NodeStatus::FAILURE;
  }

  // added 1 cm offset to the actual table height to prevent close grasps
  setOutput("table_height", segment_plane_->getTableHeight() + 0.01);
  setOutput("nonplane_cloud", segment_plane_->getNonPlaneCloud());
  setOutput("plane_cloud", segment_plane_->getPlaneCloud());
  setOutput("plane_coeff", segment_plane_->getPlaneCoeff());
  setOutput("tabletop_cloud", segment_plane_->getTableTopCloud());
  setOutput("image_scene", segment_plane_->getImage());
  setOutput("original_cloud", segment_plane_->getOriginalCloud());
  setOutput("original_cloud_transformed", segment_plane_->getOriginalTransformedCloud());

  return BT::NodeStatus::SUCCESS;
}
}
