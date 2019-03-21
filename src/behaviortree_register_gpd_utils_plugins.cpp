#include <gpd_utils/bt_actions/object_cloud_extraction_action.h>
#include <gpd_utils/bt_actions/generate_grasping_candidates_action.h>
#include <gpd_utils/bt_actions/tabletop_clustering_action.h>
#include <gpd_utils/bt_actions/tabletop_segmentation_action.h>
#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pal::GenerateGraspingCandidatesAction>("GenerateGraspingCandidates");
  factory.registerNodeType<pal::ObjectCloudExtractionAction>("ObjectCloudExtraction");
  factory.registerNodeType<pal::TableTopClusteringAction>("TableTopClustering");
  factory.registerNodeType<pal::TableTopSegmentationAction>("TableTopSegmentation");
}
