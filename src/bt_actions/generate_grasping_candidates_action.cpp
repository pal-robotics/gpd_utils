#include <gpd_utils/bt_actions/generate_grasping_candidates_action.h>

namespace pal
{
GenerateGraspingCandidatesAction::GenerateGraspingCandidatesAction(const std::string &name,
                                                                   const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config)
{
}

GenerateGraspingCandidatesAction::~GenerateGraspingCandidatesAction()
{
}

void GenerateGraspingCandidatesAction::init(const ros::NodeHandle &nh)
{
  nh_ = nh;
  ac_.reset(new actionlib::SimpleActionClient<gpd_utils::GraspCandidatesGenerationAction>(
      "/generate_grasp_candidates", true));
}

BT::NodeStatus GenerateGraspingCandidatesAction::tick()
{
  GET_INPUT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, object_cloud);
  GET_INPUT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, original_cloud_transformed);
  GET_INPUT(double, table_height);
  ac_->waitForServer(ros::Duration(5.0));

  gpd_utils::GraspCandidatesGenerationGoal goal;
  goal.table_height = table_height.value();
  for (pcl::PointXYZRGB point : object_cloud.value()->points)
  {
    geometry_msgs::Point sample_point;
    sample_point.x = point.x;
    sample_point.y = point.y;
    sample_point.z = point.z;
    goal.samples.push_back(sample_point);
  }
  pcl::toROSMsg(*original_cloud_transformed.value(), goal.pointcloud);

  ROS_INFO("Sending the pointcloud to generate the grasping candidates!");
  actionlib::SimpleClientGoalState state = ac_->sendGoalAndWait(goal, ros::Duration(25.0));
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    return BT::NodeStatus::FAILURE;

  gpd_utils::GraspCandidatesGenerationResult result;
  result = *ac_->getResult();
  ROS_INFO_STREAM("GPD : Generated a total of " << result.grasp_candidates.size()
                                                << " grasp candidates");

  setOutput("grasp_candidates", result.grasp_candidates);

  return BT::NodeStatus::SUCCESS;
}
}
