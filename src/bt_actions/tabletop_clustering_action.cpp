#include <gpd_utils/bt_actions/tabletop_clustering_action.h>

namespace pal
{
TableTopClusteringAction::TableTopClusteringAction(const std::string &name,
                                                   const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config)
{
}

TableTopClusteringAction::~TableTopClusteringAction()
{
}

void TableTopClusteringAction::init(const ros::NodeHandle &nh)
{
  nh_ = nh;
  TTD_.reset(new pal::TableTopDetector<pcl::PointXYZRGB>(nh_));
}

BT::NodeStatus TableTopClusteringAction::tick()
{
  GET_INPUT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, tabletop_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  TTD_->extractOneCluster(tabletop_cloud.value(), object_cloud);
  setOutput("object_cloud", object_cloud);
  return BT::NodeStatus::SUCCESS;
}
}
