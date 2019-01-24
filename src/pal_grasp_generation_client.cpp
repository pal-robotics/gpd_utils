#include <gpd_utils/pal_grasp_generation_client.h>
using namespace pal;

PalGraspGenerationClient::PalGraspGenerationClient(ros::NodeHandle& node)
  : has_cloud_(false), nh_(node), grasp_generation_client_("/generate_grasp_candidates", true)
{
  cloud_sub_ = nh_.subscribe("/segment_table/nonplane", 1,
                             &PalGraspGenerationClient::cloudCallback, this);
  ROS_INFO("Waiting for Grasp Generation Action Server to start.");
  grasp_generation_client_.waitForServer();
  ROS_INFO("Grasp Generation Action Server started, sending goal.");
}


void PalGraspGenerationClient::run()
{
  ros::Rate rate(100);

  while (ros::ok())
  {
    if (has_cloud_)
    {
      gpd_utils::GraspCandidatesGenerationGoal goal;
      goal.pointcloud = cloud_data_;
      grasp_generation_client_.sendGoalAndWait(goal);

      gpd_utils::GraspCandidatesGenerationResult result;
      result = *grasp_generation_client_.getResult();
      ROS_INFO_STREAM("Received " << result.grasp_candidates.size()
                                  << " grasp candidate from the point cloud");

      // Reset the system.
      has_cloud_ = false;
      ROS_INFO("Waiting for point cloud to arrive ...");
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void PalGraspGenerationClient::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!has_cloud_)
  {
    cloud_data_ = *msg;
    has_cloud_ = true;
  }
}

int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "pal_grasp_generation_client");
  ros::NodeHandle node("~");

  PalGraspGenerationClient grasp_detection(node);
  grasp_detection.run();

  return 0;
}
