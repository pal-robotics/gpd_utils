/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef OBJECT_RECOGNITION_INFO_H
#define OBJECT_RECOGNITION_INFO_H

// PAL headers
#include <pal_pcl/pcl_filters.hpp>
#include <pal_detection_msgs/RecognizeObjectsAction.h>
#include <pal_detection_msgs/RecognizedObjectArray.h>
#include <gpd_utils/object_bounding_box.h>

// PCL headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

namespace pal
{
class ObjectRecognitionInfo
{
public:
  ObjectRecognitionInfo(ros::NodeHandle &nh, const ros::Duration &timeout = ros::Duration(0,0));

  virtual ~ObjectRecognitionInfo();

  bool computeObjectBoundingBox(const std::string &object_name, gpd_utils::BoundingBox &bbox);

  void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud);

  void setImage(const sensor_msgs::CompressedImagePtr &image);

private:
  void pixelTo3DPoint(int u, int v, geometry_msgs::PointStamped &p) const;

  void recognizedObjectsInfo(const pal_detection_msgs::RecognizedObjectArray &recognized_objects,
                             std::vector<std::string> &classes,
                             std::vector<sensor_msgs::RegionOfInterest> &bboxes) const;

  void transformPoint(const std::string &frame_id, geometry_msgs::PointStamped &point_out) const;

  void computeBBoxPoints(int &xmin, int &ymin, int &xmax, int &ymax,
                         gpd_utils::BoundingBox &bbox) const;

  bool isOverlap(int object_index, const std::vector<sensor_msgs::RegionOfInterest> &BBoxes,
                 const std::vector<std::string> &classes) const;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
  sensor_msgs::CompressedImagePtr image_;

  ros::NodeHandle nh_;

  tf::TransformListener _tfListener;

  // frame in which the point cloud will be transformed and processed
  std::string out_frame_;

  actionlib::SimpleActionClient<pal_detection_msgs::RecognizeObjectsAction> ac_;
  std::string desired_object_;
  bool received_data_;

  std::vector<gpd_utils::BoundingBox> bb_filter_indices_;
  gpd_utils::BoundingBox bbox_;
  int bb_padding_;

  ros::Publisher lu_pub_, rb_pub_, mid_pub_;
};
}

#endif  // OBJECT_RECOGNITION_INFO_H
