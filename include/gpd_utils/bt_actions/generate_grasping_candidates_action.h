#ifndef GENERATE_GRASPING_CANDIDATES_ACTION_H
#define GENERATE_GRASPING_CANDIDATES_ACTION_H

#include <behaviortree_ros_actions/behaviortree_pal_utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <gpd_utils/GraspCandidatesGenerationAction.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pal
{
class GenerateGraspingCandidatesAction : public BT::SyncActionNode
{
public:
  GenerateGraspingCandidatesAction(const std::string &name, const BT::NodeConfiguration &config);

  virtual ~GenerateGraspingCandidatesAction();

  static BT::PortsList providedPorts()
  {
    static const BT::PortsList ports = {
      { BT::InputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
            "object_cloud", "Object's pointcloud information"),
        BT::InputPort<double>("table_height", "The height of the table on which the object is present"),
        BT::InputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
            "original_cloud_transformed", "Transformed Original Pointcloud Data"),
        BT::InputPort<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(
            "grasp_candidates", "The grasp candidates around the object's cloud") }
    };

    return ports;
  }

  void init(const ros::NodeHandle &nh);

  virtual BT::NodeStatus tick() override;

protected:
  ros::NodeHandle nh_;
  std::unique_ptr<actionlib::SimpleActionClient<gpd_utils::GraspCandidatesGenerationAction>> ac_;
};
}

#endif  // GENERATE_GRASPING_CANDIDATES_ACTION_H
