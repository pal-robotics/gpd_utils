/*
  @file

  @author Sai Kishor Kothakota

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>
#include <gpd_utils/segment_table.h>
#include <gpd_utils/tabletop_detector_class.h>

namespace pal
{
TEST(GraspCandidatesEvaluationTest, graspCandidatesTest)
{
  ros::NodeHandle nh, pnh("~");
  std::string pack_path = ros::package::getPath("gpd_utils");
  std::map<std::string, double> rosbags_param;
  std::map<std::string, double> cluster_info_param;
  double table_height = 0.470;
  pnh.getParam("segmentation_bags", rosbags_param);
  pnh.getParam("segmentation_bags_cluster_info", cluster_info_param);

  pal::PlanarSegmentationParams params;
  params.readConfig<ariles::ros>(nh, "/PlanarSegmentationParams");
  PlanarSegmentation<pcl::PointXYZRGB> segment_plane(nh, pnh, params);
  pal::TableTopDetector<pcl::PointXYZRGB> TTD(nh);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ASSERT_FALSE(TTD.extractOneCluster(empty_cloud, object_cloud));
  for (auto it = rosbags_param.begin(); it != rosbags_param.end(); it++)
  {
    std::string bag_path = pack_path + "/test/segmentation_bags/" + it->first;
    SCOPED_TRACE("Bag : " + bag_path);
    ASSERT_TRUE(boost::filesystem::exists(bag_path)) << bag_path;
    rosbag::Bag rbag;
    rbag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(rbag);
    sensor_msgs::PointCloud2ConstPtr cloud_msg;
    sensor_msgs::CompressedImageConstPtr image_msg;
    for (rosbag::MessageInstance const &m : view)
    {
      if (!cloud_msg.get())
        cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (!image_msg.get())
        image_msg = m.instantiate<sensor_msgs::CompressedImage>();
    }
    pcl::PointCloud<pcl::PointXYZ> pointcloud_info = pcl::PointCloud<pcl::PointXYZ>();
    pcl::PointXYZ minPt, maxPt;
    ASSERT_TRUE(cloud_msg.get());
    if (!cloud_msg->data.empty())
    {
      pcl::fromROSMsg(*cloud_msg, pointcloud_info);
    }
    segment_plane.setCloudAndImageScene(cloud_msg, image_msg);
    ros::Duration(0.5).sleep();
    ASSERT_TRUE(segment_plane.performTableSegmentation());
    ASSERT_GT(segment_plane.getTableTopCloud()->size(), 0);
    EXPECT_NEAR(segment_plane.getTableHeight(), it->second, 0.02);
    object_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    ASSERT_TRUE(TTD.extractOneCluster(segment_plane.getTableTopCloud(), object_cloud));
    EXPECT_NEAR(object_cloud->size(), cluster_info_param.at(it->first), 1000);
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_segmentation_test");
  ros::NodeHandle nh;
  ros::Time::waitForValid();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
