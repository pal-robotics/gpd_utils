#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <gpd_utils/GraspCandidatesGenerationAction.h>

namespace pal
{
void expectPoseEquality(const geometry_msgs::PoseArray &expected_poses,
                        const std::vector<geometry_msgs::PoseStamped> &actual_poses)
{
  ASSERT_EQ(expected_poses.poses.size(), actual_poses.size());
  for (size_t i = 0; i < actual_poses.size(); ++i)
  {
    EXPECT_NEAR(expected_poses.poses.at(i).position.x, actual_poses.at(i).pose.position.x, 0.01);
    EXPECT_NEAR(expected_poses.poses.at(i).position.y, actual_poses.at(i).pose.position.y, 0.01);
    EXPECT_NEAR(expected_poses.poses.at(i).position.z, actual_poses.at(i).pose.position.z, 0.01);
    EXPECT_NEAR(expected_poses.poses.at(i).orientation.x,
                actual_poses.at(i).pose.orientation.x, 0.001);
    EXPECT_NEAR(expected_poses.poses.at(i).orientation.y,
                actual_poses.at(i).pose.orientation.y, 0.001);
    EXPECT_NEAR(expected_poses.poses.at(i).orientation.z,
                actual_poses.at(i).pose.orientation.z, 0.001);
    EXPECT_NEAR(expected_poses.poses.at(i).orientation.w,
                actual_poses.at(i).pose.orientation.w, 0.001);
  }
}

void expectCandidatesInCloudVicinity(const std::vector<geometry_msgs::PoseStamped> &actual_poses,
                                     const pcl::PointXYZ &minPt, const pcl::PointXYZ &maxPt,
                                     const double threshold = 0.05)
{
  for (geometry_msgs::PoseStamped pose_st : actual_poses)
  {
    EXPECT_GE(pose_st.pose.position.x, minPt.x - threshold);
    EXPECT_LE(pose_st.pose.position.x, maxPt.x + threshold);
    EXPECT_GE(pose_st.pose.position.y, minPt.y - threshold);
    EXPECT_LE(pose_st.pose.position.y, maxPt.y + threshold);
    EXPECT_GE(pose_st.pose.position.z, minPt.z - threshold);
    EXPECT_LE(pose_st.pose.position.z, maxPt.z + threshold);
  }
}

TEST(GraspCandidatesEvaluationTest, graspCandidatesTest)
{
  ros::NodeHandle nh("~");
  std::string pack_path = ros::package::getPath("gpd_utils");
  actionlib::SimpleActionClient<gpd_utils::GraspCandidatesGenerationAction> grasp_generation_client_(
      "/generate_grasp_candidates", true);
  std::vector<std::string> rosbags;
  double table_height = 0.470;
  ASSERT_TRUE(nh.getParam("object_cloud_rosbag_files", rosbags));
  for (std::string bag : rosbags)
  {
    std::string bag_path = pack_path + "/test/rosbags/" + bag;
    SCOPED_TRACE("Bag : " + bag_path);
    ASSERT_TRUE(boost::filesystem::exists(bag_path)) << bag_path;
    rosbag::Bag rbag;
    rbag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(rbag);

    geometry_msgs::PoseArray expected_candidates = geometry_msgs::PoseArray();
    sensor_msgs::PointCloud2ConstPtr cloud_msg;
    geometry_msgs::PoseArrayConstPtr candidates_msg;
    for (rosbag::MessageInstance const &m : view)
    {
      if (!cloud_msg.get())
        cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (!candidates_msg.get())
        candidates_msg = m.instantiate<geometry_msgs::PoseArray>();
    }
    pcl::PointCloud<pcl::PointXYZ> pointcloud_info = pcl::PointCloud<pcl::PointXYZ>();
    pcl::PointXYZ minPt, maxPt;
    ASSERT_TRUE(cloud_msg.get());
    if (!cloud_msg->data.empty())
    {
      ASSERT_TRUE(candidates_msg.get());
      expected_candidates = *candidates_msg;
      pcl::fromROSMsg(*cloud_msg, pointcloud_info);
      pcl::getMinMax3D(pointcloud_info, minPt, maxPt);
    }

    ASSERT_TRUE(grasp_generation_client_.waitForServer(ros::Duration(2.0)));
    ROS_INFO_STREAM("Performing test on the cloud with " << cloud_msg->data.size() << " points");
    gpd_utils::GraspCandidatesGenerationGoal goal;
    goal.pointcloud = *cloud_msg;
    goal.table_height = table_height;
    ROS_INFO_STREAM("Sending goal and waiting for the result");
    grasp_generation_client_.sendGoalAndWait(goal);
    gpd_utils::GraspCandidatesGenerationResult result =
        gpd_utils::GraspCandidatesGenerationResult();
    result = *grasp_generation_client_.getResult();
    if (!cloud_msg->data.empty())
    {
      ASSERT_GT(result.grasp_candidates.size(), 0);
    }
    else
    {
      ASSERT_TRUE(result.grasp_candidates.empty());
    }
    EXPECT_GE(result.grasp_candidates.size(), floor(expected_candidates.poses.size() / 3.0));
    // expectPoseEquality(expected_candidates, result.grasp_candidates);
    expectCandidatesInCloudVicinity(result.grasp_candidates, minPt, maxPt, 0.11);
  }
}

TEST(GraspCandidatesEvaluationTest, graspObjectCandidatesTest)
{
  ros::NodeHandle nh("~");
  std::string pack_path = ros::package::getPath("gpd_utils");
  actionlib::SimpleActionClient<gpd_utils::GraspCandidatesGenerationAction> grasp_generation_client_(
      "/generate_grasp_candidates", true);
  std::vector<std::string> rosbags;
  double table_height = 0.470;
  ASSERT_TRUE(nh.getParam("original_cloud_rosbag_files", rosbags));
  for (std::string bag : rosbags)
  {
    geometry_msgs::PoseArray expected_candidates = geometry_msgs::PoseArray();
    sensor_msgs::PointCloud2ConstPtr object_cloud_msg;
    sensor_msgs::PointCloud2ConstPtr orig_cloud_msg;
    geometry_msgs::PoseArrayConstPtr candidates_msg;

    std::string bag_path = pack_path + "/test/rosbags/" + bag;
    SCOPED_TRACE("Bag : " + bag_path);
    ASSERT_TRUE(boost::filesystem::exists(bag_path)) << bag_path;
    rosbag::Bag rbag;
    rbag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(rbag);

    for (rosbag::MessageInstance const &m : view)
    {
      if (!candidates_msg.get())
        candidates_msg = m.instantiate<geometry_msgs::PoseArray>();
    }
    std::vector<std::string> cloud_topic{ "/object_cloud" };
    rosbag::View view_object(rbag, rosbag::TopicQuery(cloud_topic));
    for (rosbag::MessageInstance const &m : view_object)
    {
      if (!object_cloud_msg.get())
        object_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
    }
    cloud_topic.clear();
    cloud_topic.push_back("/original_cloud");
    rosbag::View view_original(rbag, rosbag::TopicQuery(cloud_topic));
    for (rosbag::MessageInstance const &m : view_object)
    {
      if (!orig_cloud_msg.get())
        orig_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
    }

    ASSERT_TRUE(object_cloud_msg.get());
    ASSERT_TRUE(orig_cloud_msg.get());

    ASSERT_TRUE(grasp_generation_client_.waitForServer(ros::Duration(2.0)));
    ROS_INFO_STREAM("Performing test on the cloud with " << orig_cloud_msg->data.size()
                                                         << " points");
    pcl::PointXYZ minPt, maxPt;
    gpd_utils::GraspCandidatesGenerationGoal goal;
    if (!object_cloud_msg->data.empty())
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_info(new pcl::PointCloud<pcl::PointXYZ>());
      ASSERT_TRUE(candidates_msg.get());
      expected_candidates = *candidates_msg;
      pcl::fromROSMsg(*object_cloud_msg, *pointcloud_info);
      for (pcl::PointXYZ point : pointcloud_info->points)
      {
        geometry_msgs::Point sample_point;
        sample_point.x = point.x;
        sample_point.y = point.y;
        sample_point.z = point.z;
        goal.samples.push_back(sample_point);
      }
      pcl::getMinMax3D(*pointcloud_info, minPt, maxPt);
      ASSERT_GT(goal.samples.size(), 0);
    }

    goal.pointcloud = *object_cloud_msg;
    goal.table_height = table_height;
    ROS_INFO_STREAM("Sending goal and waiting for the result");
    grasp_generation_client_.sendGoalAndWait(goal);
    gpd_utils::GraspCandidatesGenerationResult result =
        gpd_utils::GraspCandidatesGenerationResult();
    result = *grasp_generation_client_.getResult();
    if (!object_cloud_msg->data.empty())
    {
      ASSERT_GT(result.grasp_candidates.size(), 0);
    }
    else
    {
      ASSERT_TRUE(result.grasp_candidates.empty());
    }
    EXPECT_GE(result.grasp_candidates.size(), floor(expected_candidates.poses.size() / 5.0));
    expectCandidatesInCloudVicinity(result.grasp_candidates, minPt, maxPt, 0.11);
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpd_candidates_testing");
  ros::NodeHandle nh("~");
  ros::Time::waitForValid();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
