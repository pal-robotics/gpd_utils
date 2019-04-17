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
  auto cluster = TTD_->extractOneCluster(tabletop_cloud.value());
  if (!cluster.get() || cluster->empty())
    return BT::NodeStatus::FAILURE;
  setOutput("object_cloud", cluster);
  return BT::NodeStatus::SUCCESS;
}
}
