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
#include <gmock/gmock.h>
#include <geometry_msgs/PoseStamped.h>
#include <gpd_utils/object_recognition_info.h>
#include <gpd_utils/tabletop_detector_class.h>
#include <pal_detection_msgs/RecognizeObjectsAction.h>
#include <pal_test_utils/mock_action_server.h>

using ::testing::InvokeWithoutArgs;
using ::testing::_;

namespace pal
{
TEST(ObjectRecognitionInfoTest, pointCloudCroppingTest)
{
  ros::NodeHandle nh, pnh("~");
  std::string pack_path = ros::package::getPath("gpd_utils");
  std::map<std::string, double> rosbags_param;
  double table_height = 0.470;
  std::string desired_object = "coke";
  pnh.getParam("object_recognition_info", rosbags_param);
  tf::TransformListener tfListener;
  ros::Duration(1.0).sleep();
  for (auto it = rosbags_param.begin(); it != rosbags_param.end(); it++)
  {
    MockActionServer<pal_detection_msgs::RecognizeObjectsAction> inference_as("/inference_server");
    ObjectRecognitionInfo obj_info(nh);
    pal::TableTopDetector<pcl::PointXYZRGB> TTD(nh);
    std::string bag_path = pack_path + "/test/object_recog_bags/" + it->first;
    SCOPED_TRACE("Bag : " + bag_path);
    ASSERT_TRUE(boost::filesystem::exists(bag_path)) << bag_path;
    rosbag::Bag rbag;
    rbag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(rbag);
    sensor_msgs::PointCloud2ConstPtr object_cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud_msg, tabletopcloud_msg;
    sensor_msgs::CompressedImagePtr image_msg;
    pal_detection_msgs::RecognizeObjectsActionResultConstPtr action_result;
    for (rosbag::MessageInstance const &m : view)
    {
      if (!action_result.get())
        action_result = m.instantiate<pal_detection_msgs::RecognizeObjectsActionResult>();
      sensor_msgs::CompressedImageConstPtr msg = m.instantiate<sensor_msgs::CompressedImage>();
      if (msg.get())
        image_msg.reset(new sensor_msgs::CompressedImage(*msg));
    }
    std::vector<std::string> cloud_topic{ "/object_cloud" };
    rosbag::View view_object(rbag, rosbag::TopicQuery(cloud_topic));
    for (rosbag::MessageInstance const &m : view_object)
    {
      sensor_msgs::PointCloud2ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      object_cloud_msg.reset(new sensor_msgs::PointCloud2(*cloud_msg));
    }
    cloud_topic.clear();
    cloud_topic.push_back("/original_cloud");
    rosbag::View view_original(rbag, rosbag::TopicQuery(cloud_topic));
    for (rosbag::MessageInstance const &m : view_original)
    {
      sensor_msgs::PointCloud2ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      original_cloud_msg.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

      if (!cloud_msg->data.empty() && cloud_msg.get())
      {
        pcl::fromROSMsg(*cloud_msg, *original_cloud_msg);
      }
      ros::Duration(0.1).sleep();
      // To change the pointcloud frame back to xtion_rgb_optical_frame
      bool transform_complete = pcl_ros::transformPointCloud(
          "xtion_rgb_optical_frame", ros::Time(0), *original_cloud_msg,
          original_cloud_msg->header.frame_id, *original_cloud_msg, tfListener);
      original_cloud_msg->header.frame_id = "xtion_rgb_optical_frame";
      pcl_conversions::toPCL(ros::Time::now(), original_cloud_msg->header.stamp);
    }
    cloud_topic.clear();
    cloud_topic.push_back("/tabletop_cloud");
    rosbag::View view_tabletop(rbag, rosbag::TopicQuery(cloud_topic));
    for (rosbag::MessageInstance const &m : view_tabletop)
    {
      sensor_msgs::PointCloud2ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      tabletopcloud_msg.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
      if (!cloud_msg->data.empty())
      {
        pcl::fromROSMsg(*cloud_msg, *tabletopcloud_msg);
      }
      tabletopcloud_msg->header.stamp = original_cloud_msg->header.stamp;
    }
    ROS_INFO("The width and the height of the image is %d, %d", original_cloud_msg->width,
             original_cloud_msg->height);
    obj_info.setPointCloud(original_cloud_msg);
    obj_info.setImage(image_msg);

    EXPECT_CALL(inference_as, goalReceived(_)).Times(1).WillOnce(InvokeWithoutArgs([&]() {
      inference_as.setSucceeded(action_result->result);
    }));
    gpd_utils::BoundingBox bbox;
    ASSERT_TRUE(obj_info.computeObjectBoundingBox(desired_object, bbox));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud =
        TTD.getObjectCloud(tabletopcloud_msg, bbox);
    ASSERT_NEAR(clustered_cloud->size(), object_cloud_msg->width * object_cloud_msg->height, 100);
    EXPECT_EQ(original_cloud_msg->header.stamp, clustered_cloud->header.stamp);
    EXPECT_EQ("base_footprint", clustered_cloud->header.frame_id);
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_recognition_info_test");
  ros::NodeHandle nh;
  ros::Time::waitForValid();
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
