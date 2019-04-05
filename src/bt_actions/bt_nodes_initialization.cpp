/*
  @file

  @author victor

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/
#include <gpd_utils/bt_actions/bt_nodes_initialization.h>
#include <gpd_utils/bt_actions/generate_grasping_candidates_action.h>
#include <gpd_utils/bt_actions/object_cloud_extraction_action.h>
#include <gpd_utils/bt_actions/tabletop_clustering_action.h>
#include <gpd_utils/bt_actions/tabletop_segmentation_action.h>
namespace pal
{
void initGPDBTNodes(BT::Tree &tree, ros::NodeHandle nh, ros::NodeHandle priv_nh,
                      const pal::PlanarSegmentationParams &segm_params)
{
  // Iterate through all the nodes and call init() if it is an Action_B
  for (auto &node : tree.nodes)
  {
    if (auto grasp_cand_node = dynamic_cast<pal::GenerateGraspingCandidatesAction *>(node.get()))
    {
      grasp_cand_node->init(nh);
    }
    else if (auto obj_cloud_node = dynamic_cast<pal::ObjectCloudExtractionAction *>(node.get()))
    {
      obj_cloud_node->init(nh);
    }
    else if (auto tt_clustering_node = dynamic_cast<pal::TableTopClusteringAction *>(node.get()))
    {
      tt_clustering_node->init(nh);
    }
    else if (auto tt_segm_node = dynamic_cast<pal::TableTopSegmentationAction *>(node.get()))
    {
      tt_segm_node->init(nh, priv_nh, segm_params);
    }
  }
}
}