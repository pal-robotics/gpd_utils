#include <gpd_utils/states/generate_grasping_candidates_state.h>

using namespace pal;

GenerateGraspingCandidatesState::GenerateGraspingCandidatesState(const ros::NodeHandle &nh)
  : nh_(nh)
  , ac_("/generate_grasp_candidates", true)
  , State({ smach_c::SUCCESS, smach_c::PREEMPTED, smach_c::FAILURE })
{
  ROS_INFO("Waiting for the grasp generation server - GPD");
  ac_.waitForServer();
  ROS_INFO("Grasp generation server - GPD Started!!");
}

GenerateGraspingCandidatesState::~GenerateGraspingCandidatesState()
{
}

std::string GenerateGraspingCandidatesState::execute(smach_c::UserData &user_data)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  double table_height;

  try
  {
    user_data.getPropertyValue("~object_cloud", object_cloud,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~table_height", table_height,
                               property_bag::RetrievalHandling::THROW);
    user_data.getPropertyValue("~original_cloud_transformed", original_cloud_transformed,
                               property_bag::RetrievalHandling::THROW);
  }
  catch (property_bag::PropertyException)
  {
    ROS_ERROR_STREAM(
        "Unable to obtain the object cloud, original transformed cloud and the table_height info from the propert_bag");
    return smach_c::FAILURE;
  }
  gpd_utils::GraspCandidatesGenerationGoal goal;
  goal.table_height = table_height;
  for (pcl::PointXYZRGB point : object_cloud->points)
  {
    geometry_msgs::Point sample_point;
    sample_point.x = point.x;
    sample_point.y = point.y;
    sample_point.z = point.z;
    goal.samples.push_back(sample_point);
  }
  pcl::toROSMsg(*original_cloud_transformed, goal.pointcloud);

  ROS_INFO("Sending the pointcloud to generate the grasping candidates!");
  ac_.sendGoalAndWait(goal, ros::Duration(25.0));

  gpd_utils::GraspCandidatesGenerationResult result;
  result = *ac_.getResult();
  ROS_INFO_STREAM("GPD : Generated a total of " << result.grasp_candidates.size()
                                                << " grasp candidates");

  user_data.addOrUpdateProperty("~grasp_candidates", result.grasp_candidates);
  return smach_c::SUCCESS;
}
