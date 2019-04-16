#include <gpd_utils/bt_actions/tabletop_clustering_action.h>

namespace pal
{
TableTopClusteringAction::TableTopClusteringAction(const std::string &name,
                                                   const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config)
{
  ros::NodeHandle nh;
  init(nh);
}

TableTopClusteringAction::~TableTopClusteringAction()
{
}

void TableTopClusteringAction::init(ros::NodeHandle nh)
{
  TTD_.reset(new pal::TableTopDetector<pcl::PointXYZRGB>(nh));
}

BT::NodeStatus TableTopClusteringAction::tick()
{
  GET_INPUT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, tabletop_cloud);
  setOutput("object_cloud", TTD_->extractOneCluster(tabletop_cloud.value()));
  return BT::NodeStatus::SUCCESS;
}
}
