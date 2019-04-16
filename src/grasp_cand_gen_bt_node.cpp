#include <behaviortree_cpp/bt_factory.h>
#include <gpd_utils/bt_actions/generate_grasping_candidates_action.h>
#include <gpd_utils/bt_actions/object_cloud_extraction_action.h>
#include <gpd_utils/bt_actions/tabletop_clustering_action.h>
#include <gpd_utils/bt_actions/tabletop_segmentation_action.h>
#include <gpd_utils/bt_actions/bt_nodes_initialization.h>
#include <behaviortree_ros_actions/blackboard_entry_check.h>
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "table_top_grasping_candidates_node");
  ros::NodeHandle nh, pnh("~");

  pal::PlanarSegmentationParams params;
  params.readConfig<ariles::ros>(nh, "/PlanarSegmentationParams");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("desired_object", "coke");
  const std::string xml_path =
      ros::package::getPath("gpd_utils") + "/config/bt_trees/simple_tree.xml";

  BT::BehaviorTreeFactory factory;
  factory.registerFromROSPlugins();
  factory.registerNodeType<pal::BlackboardEntryCheck<std::string>>("BlackboardEntryCheck");
  auto tree = factory.createTreeFromFile(xml_path, blackboard);
  pal::initGPDBTNodes(tree, nh, pnh, params);
  tree.root_node->executeTick();
  ros::spin();
}
