#include <gpd_utils/states/tabletop_segmentation_state.h>
#include <gpd_utils/states/generate_grasping_candidates_state.h>
#include <gpd_utils/states/object_cloud_extraction_state.h>
#include <smach_c/state_machine_with_introspection.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_grasp_candidates_node");

  ros::NodeHandle nh, pnh("~");

  ros::Publisher object_cloud_pub =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("object_cloud", 1, true);
  ros::Publisher orig_cloud_pub =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("original_cloud", 1, true);
  ros::Publisher grasp_candidates_pub =
      nh.advertise<geometry_msgs::PoseArray>("grasp_candiates", 1, true);

  pal::PlanarSegmentationParams params;
  params.readConfig<ariles::ros>(nh, "/PlanarSegmentationParams");

  smach_c::UserData user_data;

  user_data.addProperty("desired_object", std::string("coke"));

  // Creating a SMACH state machine
  ROS_INFO("Initializing the state machine");
  smach_c::StateMachineWithIntrospectionPtr sm(new smach_c::StateMachineWithIntrospection(
      nh, { "TASK_COMPLETE", "TASK_INCOMPLETE", "TASK_PREEMPTED" }));

  sm->add("TableTop Segmentation",
          smach_c::StatePtr(new pal::TableTopSegmentationState(nh, pnh, params)),
          { { smach_c::PREEMPTED, "TASK_PREEMPTED" },
            { smach_c::SUCCESS, "Object Cloud Extraction" },
            { smach_c::FAILURE, "TASK_INCOMPLETE" } },
          { { "~table_height", "table_height" },
            { "~nonplane_cloud", "nonplane_cloud" },
            { "~plane_cloud", "plane_cloud" },
            { "~plane_coeff", "plane_coeff" },
            { "~tabletop_cloud", "tabletop_cloud" },
            { "~image_scene", "image_scene" },
            { "~original_cloud", "original_cloud" },
            { "~original_cloud_transformed", "original_cloud_transformed" } });

  sm->add("Object Cloud Extraction", smach_c::StatePtr(new pal::ObjectCloudExtractionState(nh)),
          { { smach_c::PREEMPTED, "TASK_PREEMPTED" },
            { smach_c::SUCCESS, "Grasp Candidates" },
            { smach_c::FAILURE, "TASK_INCOMPLETE" } },
          { { "~object_cloud", "object_cloud" },
            { "~desired_object", "desired_object" },
            { "~tabletop_cloud", "tabletop_cloud" },
            { "~image_scene", "image_scene" },
            { "~original_cloud", "original_cloud" } });

  sm->add("Grasp Candidates", smach_c::StatePtr(new pal::GenerateGraspingCandidatesState(nh)),
          { { smach_c::PREEMPTED, "TASK_PREEMPTED" },
            { smach_c::SUCCESS, "TASK_COMPLETE" },
            { smach_c::FAILURE, "TASK_INCOMPLETE" } },
          { { "~object_cloud", "object_cloud" },
            { "~original_cloud_transformed", "original_cloud_transformed" },
            { "~table_height", "table_height" },
            { "~grasp_candidates", "grasp_candidates" } });
  sm->execute(user_data);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, clustered_cloud;
  user_data.getPropertyValue("object_cloud", clustered_cloud);
  user_data.getPropertyValue("original_cloud_transformed", original_cloud);

  std::vector<geometry_msgs::PoseStamped> grasp_cand;
  user_data.getPropertyValue("grasp_candidates", grasp_cand);

  geometry_msgs::PoseArray grasp_candidates_pose = geometry_msgs::PoseArray();
  if (grasp_cand.size() > 0)
  {
    grasp_candidates_pose.header = grasp_cand.at(0).header;
    for (auto grasp : grasp_cand)
      grasp_candidates_pose.poses.push_back(grasp.pose);
  }

  ROS_INFO_STREAM("Clustered Cloud is of size : " << clustered_cloud->height *
                                                         clustered_cloud->width);
  object_cloud_pub.publish(*clustered_cloud);
  orig_cloud_pub.publish(*original_cloud);
  ROS_INFO_STREAM("A total of "
                  << grasp_cand.size()
                  << " grasp candidates has been generated from the clustered cloud");
  grasp_candidates_pub.publish(grasp_candidates_pose);

  ros::spin();
}
