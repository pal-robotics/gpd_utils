/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef SEGMENT_TABLE_H
#define SEGMENT_TABLE_H

// PAL headers
#include <pal_pcl/pcl_filters.hpp>
#include <gpd_utils/segment_table_params.h>
// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL headers
#include <pcl/point_cloud.h>

// Std C++ headers
#include <string>

namespace pal
{

template <class PointT>
class PlanarSegmentation
{
public:
  PlanarSegmentation(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                     const PlanarSegmentationParams& params);

  virtual ~PlanarSegmentation();

  bool performTableSegmentation();

  typename pcl::PointCloud<PointT>::Ptr getOriginalCloud() const;

  typename pcl::PointCloud<PointT>::Ptr getOriginalTransformedCloud() const;

  sensor_msgs::CompressedImagePtr getImage() const;

  typename pcl::PointCloud<PointT>::Ptr getTableTopCloud() const;

  typename pcl::PointCloud<PointT>::Ptr getPlaneCloud() const;

  typename pcl::PointCloud<PointT>::Ptr getNonPlaneCloud() const;

  pcl::ModelCoefficients::Ptr getPlaneCoeff() const;

  double getTableHeight() const;

  void setCloudAndImageScene(const sensor_msgs::PointCloud2ConstPtr& cloud,
                             const sensor_msgs::CompressedImageConstPtr& image);

protected:
  void cloudImageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                          const sensor_msgs::CompressedImageConstPtr& image);

  bool processCloudData();

  void getCloudData();

  void publish(typename pcl::PointCloud<PointT>::Ptr& planeCloud,
               typename pcl::PointCloud<PointT>::Ptr& nonPlaneCloud,
               typename pcl::PointCloud<PointT>::Ptr& tableTopCloud,
               pcl::ModelCoefficients::Ptr& planeCoeff, pcl::uint64_t& stamp,
               const std::string& frameId) const;

  void publishEmptyClouds(pcl::uint64_t& stamp, const std::string& frameId) const;

  ros::NodeHandle _nh, _pnh;
  ros::CallbackQueue _cbQueue;

  tf::TransformListener _tfListener;

  // planar segmentation parameters
  PlanarSegmentationParams params_;

  // height of the table
  double table_height_;

  // ROS publishers
  ros::Publisher plane_cloud_pub_;
  ros::Publisher nonplane_cloud_pub_;
  ros::Publisher table_top_cloud_pub_;
  ros::Publisher plane_coeff_pub_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> msg_filters_policy;
  boost::scoped_ptr<message_filters::Synchronizer<msg_filters_policy> > image_cloud_sync_;

  bool has_cloud_;
  typename pcl::PointCloud<PointT>::Ptr pointcloud_org_;
  typename pcl::PointCloud<PointT>::Ptr cloud_org_transformed_;
  typename pcl::PointCloud<PointT>::Ptr pcl_filtered_plane_cloud_;
  typename pcl::PointCloud<PointT>::Ptr pcl_filtered_nonplane_cloud_;
  typename pcl::PointCloud<PointT>::Ptr pcl_filtered_table_top_cloud_;
  pcl::ModelCoefficients::Ptr plane_coeff_;
  sensor_msgs::CompressedImagePtr image_org_;
};

template <class PointT>
PlanarSegmentation<PointT>::PlanarSegmentation(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                               const PlanarSegmentationParams& params)
  : _nh(nh), _pnh(pnh), has_cloud_(false), params_(params)
{
  _nh.setCallbackQueue(&_cbQueue);

  ROS_INFO_STREAM("The node will operate at maximum " << params_.rate_ << " Hz");

  if (params_.processing_frame_.empty())
    ROS_INFO("The point cloud will be filtered in its original frame");
  else
    ROS_INFO_STREAM("The point cloud will be filtered after transforming it to the "
                    << params_.processing_frame_ << " frame");

  ROS_INFO_STREAM("A passthrough filter on z axis will be applied with min: "
                  << params_.passthrough_zmin_ << " and max: " << params_.passthrough_zmax_);
  ROS_INFO_STREAM("A passthrough filter on x axis will be applied with min: "
                  << params_.passthrough_xmin_ << " and max: " << params_.passthrough_xmax_);
  ROS_INFO_STREAM("Downsampling leaf size: " << params_.downsampling_size_ << "");

  plane_cloud_pub_ = _pnh.advertise<typename pcl::PointCloud<PointT> >("debug/plane", 1, true);
  nonplane_cloud_pub_ = _pnh.advertise<typename pcl::PointCloud<PointT> >("debug/nonplane", 1, true);
  table_top_cloud_pub_ =
      _pnh.advertise<typename pcl::PointCloud<PointT> >("debug/tabletop_cloud", 1, true);
  plane_coeff_pub_ = _pnh.advertise<pcl_msgs::ModelCoefficients>("debug/plane_coeff", 1, true);

  image_cloud_sync_.reset(new message_filters::Synchronizer<msg_filters_policy>(
      msg_filters_policy(10), cloud_sub_, image_sub_));
  image_cloud_sync_->registerCallback(
      boost::bind(&PlanarSegmentation<PointT>::cloudImageCallback, this, _1, _2));
}

template <class PointT>
PlanarSegmentation<PointT>::~PlanarSegmentation()
{
}

template <class PointT>
void PlanarSegmentation<PointT>::publishEmptyClouds(pcl::uint64_t& stamp,
                                                    const std::string& frameId) const
{
  typename pcl::PointCloud<PointT>::Ptr emptyCloud(new typename pcl::PointCloud<PointT>);
  pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
  this->publish(emptyCloud, emptyCloud, emptyCloud, planeCoeff, stamp, frameId);
}

template <class PointT>
bool PlanarSegmentation<PointT>::performTableSegmentation()
{
  if (!has_cloud_)
    this->getCloudData();
  return this->processCloudData();
}

template <class PointT>
void PlanarSegmentation<PointT>::getCloudData()
{
  ros::Rate loopRate(params_.rate_);
  double halfPeriod = 0.5 * 1.0 / params_.rate_;
  cloud_sub_.subscribe(_nh, "cloud_topic", 1);
  image_sub_.subscribe(_nh, "image_topic", 1);
  while (!has_cloud_)
  {
    // check for subscriber's callbacks
    _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

    loopRate.sleep();
  }
  cloud_sub_.unsubscribe();
  image_sub_.unsubscribe();
  has_cloud_ = false;
}

template <class PointT>
bool PlanarSegmentation<PointT>::processCloudData()
{
  // Transform the point cloud to the frame specified if any
  if (!params_.processing_frame_.empty())
  {
    cloud_org_transformed_.reset(new typename pcl::PointCloud<PointT>);
    ROS_INFO_STREAM("Transforming point cloud from frame "
                    << pointcloud_org_->header.frame_id << " to frame "
                    << params_.processing_frame_);
    //        bool transform_complete =
    //        pcl_ros::transformPointCloud(params_.processing_frame_,
    //        *pointcloud_org_, *cloud_org_transformed_, _tfListener);
    ros::Time original_time;
    pcl_conversions::fromPCL(pointcloud_org_->header.stamp, original_time);
    bool transform_complete = pcl_ros::transformPointCloud(
        params_.processing_frame_, original_time, *pointcloud_org_,
        pointcloud_org_->header.frame_id, *cloud_org_transformed_, _tfListener);
    if (!transform_complete)
    {
      this->publishEmptyClouds(pointcloud_org_->header.stamp, params_.processing_frame_);
      return false;
    }
    cloud_org_transformed_->header.frame_id = params_.processing_frame_;
    cloud_org_transformed_->header.stamp = pointcloud_org_->header.stamp;
  }
  else
    cloud_org_transformed_ = pointcloud_org_;
  // cloud_org_transformed_ = boost::make_shared<typename pcl::PointCloud<PointT>
  // >(*pointcloud_org_);
  // cloud_org_transformed_.reset(new typename pcl::PointCloud<PointT>(*pointcloud_org_));

  // Apply passthrough filter
  typename pcl::PointCloud<PointT>::Ptr zpassThroughCloud(new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr passThroughCloud(new typename pcl::PointCloud<PointT>);
  std::string axis;
  axis = "z";
  pal::passThrough<PointT>(cloud_org_transformed_, axis, params_.passthrough_zmin_,
                           params_.passthrough_zmax_, zpassThroughCloud);

  if (zpassThroughCloud->empty())
  {
    // if all points get removed after the pass-through filtering just return false and
    // stop processing
    this->publishEmptyClouds(cloud_org_transformed_->header.stamp,
                             cloud_org_transformed_->header.frame_id);
    return false;
  }

  axis = "x";
  pal::passThrough<PointT>(zpassThroughCloud, axis, params_.passthrough_xmin_,
                           params_.passthrough_xmax_, passThroughCloud);

  if (passThroughCloud->empty())
  {
    // if all points get removed after the pass-through filtering just return false and
    // stop processing
    this->publishEmptyClouds(cloud_org_transformed_->header.stamp,
                             cloud_org_transformed_->header.frame_id);
    return false;
  }

  // Downsample the point cloud if required
  typename pcl::PointCloud<PointT>::Ptr pclDownSampledCloud(new typename pcl::PointCloud<PointT>);
  if (params_.downsampling_size_ > 0)
    pal::downSample<PointT>(passThroughCloud, pclDownSampledCloud, params_.downsampling_size_);
  else
    pclDownSampledCloud = passThroughCloud;

  if (pclDownSampledCloud->points.size() < 10)
  {
    ROS_INFO_STREAM("Not able to locate a plane because there are only "
                    << pclDownSampledCloud->points.size() << " points");
    this->publishEmptyClouds(cloud_org_transformed_->header.stamp,
                             cloud_org_transformed_->header.frame_id);
    return false;
  }

  // Remove main plane
  typename pcl::PointCloud<PointT>::Ptr pclPlaneCloud(new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr pclNonPlaneCloud(new typename pcl::PointCloud<PointT>);
  plane_coeff_.reset(new pcl::ModelCoefficients);
  pal::planeSegmentation<PointT>(pclDownSampledCloud, 0.005, &pclPlaneCloud,
                                 &pclNonPlaneCloud, &plane_coeff_);


  // filter outliers in the plane cloud
  pcl_filtered_plane_cloud_.reset(new typename pcl::PointCloud<PointT>);
  if (pclPlaneCloud->empty())
    pcl_filtered_plane_cloud_ = pclPlaneCloud;
  else
    pal::statisticalOutlierRemoval<PointT>(pclPlaneCloud, 25, 1.0, pcl_filtered_plane_cloud_);

  // filter outliers in the cloud not belonging to the main plane
  pcl_filtered_nonplane_cloud_.reset(new typename pcl::PointCloud<PointT>);
  if (pclNonPlaneCloud->empty())
    pcl_filtered_nonplane_cloud_ = pclNonPlaneCloud;
  else
    pal::statisticalOutlierRemoval<PointT>(pclNonPlaneCloud, 25, 1.0, pcl_filtered_nonplane_cloud_);

  // Compute the height of the table respect the ground
  PointT minPt, maxPt;
  pcl::getMinMax3D(*pcl_filtered_plane_cloud_, minPt, maxPt);
  plane_coeff_->values.push_back(minPt.z);
  plane_coeff_->values.push_back(maxPt.z);
  table_height_ = maxPt.z;
  // Apply passthrough filter to non plane cloud
  typename pcl::PointCloud<PointT>::Ptr xpassThroughCloud_non_plane(
      new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr ypassThroughCloud_non_plane(
      new typename pcl::PointCloud<PointT>);
  axis = "x";
  pal::passThrough<PointT>(pcl_filtered_nonplane_cloud_, axis, minPt.x, maxPt.x,
                           xpassThroughCloud_non_plane);

  axis = "y";
  pal::passThrough<PointT>(xpassThroughCloud_non_plane, axis, minPt.y, maxPt.y,
                           ypassThroughCloud_non_plane);

  pcl_filtered_nonplane_cloud_ = ypassThroughCloud_non_plane;

  pcl_filtered_table_top_cloud_.reset(new typename pcl::PointCloud<PointT>);
  axis = "z";
  pal::passThrough<PointT>(ypassThroughCloud_non_plane, axis, maxPt.z, INFINITY,
                           pcl_filtered_table_top_cloud_);

  if (pcl_filtered_table_top_cloud_->empty())
  {
    ROS_WARN_STREAM("The tabletop cloud seems to be empty!!");
    return false;
  }

  ROS_INFO_STREAM("Processing:");
  ROS_INFO_STREAM("\tInput cloud:                 " << cloud_org_transformed_->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tAfter pass-through:          " << passThroughCloud->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tAfter downsmapling:          " << pclDownSampledCloud->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tPoints in plane:             " << pclPlaneCloud->points.size() << " points");
  ROS_INFO_STREAM("\tNon-plane points:            " << pclNonPlaneCloud->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tOutliers in plane:           "
                  << pclPlaneCloud->points.size() - pcl_filtered_plane_cloud_->points.size()
                  << " points");
  ROS_INFO_STREAM("\tOutliers in non-plane:       "
                  << pclNonPlaneCloud->points.size() -
                         pcl_filtered_nonplane_cloud_->points.size()
                  << " points");
  ROS_INFO_STREAM("\tPlane height.                Min: " << minPt.z << " ,max: " << maxPt.z);
  ROS_INFO_STREAM("\tPlane x.                     Min x: " << minPt.x
                                                           << " ,max x: " << maxPt.x);
  ROS_INFO_STREAM("\tPlane y.                     Min y: " << minPt.y
                                                           << " ,max y: " << maxPt.y);

  publish(pcl_filtered_plane_cloud_, pcl_filtered_nonplane_cloud_,
          pcl_filtered_table_top_cloud_, plane_coeff_,
          cloud_org_transformed_->header.stamp, cloud_org_transformed_->header.frame_id);

  return true;
}

template <class PointT>
void PlanarSegmentation<PointT>::setCloudAndImageScene(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                                       const sensor_msgs::CompressedImageConstPtr& image)
{
  if (((cloud->width * cloud->height) == 0) || (image->data.size() == 0))
  {
    has_cloud_ = false;
    return;
  }
  pointcloud_org_.reset(new typename pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*cloud, *pointcloud_org_);
  image_org_ = boost::make_shared<sensor_msgs::CompressedImage>(*image);
  has_cloud_ = true;
}

template <class PointT>
void PlanarSegmentation<PointT>::cloudImageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                                    const sensor_msgs::CompressedImageConstPtr& image)
{
  this->setCloudAndImageScene(cloud, image);
}

template <class PointT>
void PlanarSegmentation<PointT>::publish(typename pcl::PointCloud<PointT>::Ptr& planeCloud,
                                         typename pcl::PointCloud<PointT>::Ptr& nonPlaneCloud,
                                         typename pcl::PointCloud<PointT>::Ptr& tableTopCloud,
                                         pcl::ModelCoefficients::Ptr& planeCoeff,
                                         pcl::uint64_t& stamp, const std::string& frameId) const
{
  planeCloud->header.stamp = stamp;
  planeCloud->header.frame_id = frameId;
  plane_cloud_pub_.publish(planeCloud);

  nonPlaneCloud->header.stamp = stamp;
  nonPlaneCloud->header.frame_id = frameId;
  nonplane_cloud_pub_.publish(nonPlaneCloud);

  tableTopCloud->header.stamp = stamp;
  tableTopCloud->header.frame_id = frameId;
  table_top_cloud_pub_.publish(tableTopCloud);

  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*planeCoeff, ros_coefficients);
  pcl_conversions::fromPCL(planeCloud->header, ros_coefficients.header);
  plane_coeff_pub_.publish(ros_coefficients);
}

template <class PointT>
sensor_msgs::CompressedImagePtr PlanarSegmentation<PointT>::getImage() const
{
  return image_org_;
}

template <class PointT>
double PlanarSegmentation<PointT>::getTableHeight() const
{
  return table_height_;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PlanarSegmentation<PointT>::getNonPlaneCloud() const
{
  return pcl_filtered_nonplane_cloud_;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PlanarSegmentation<PointT>::getPlaneCloud() const
{
  return pcl_filtered_plane_cloud_;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PlanarSegmentation<PointT>::getTableTopCloud() const
{
  return pcl_filtered_table_top_cloud_;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PlanarSegmentation<PointT>::getOriginalCloud() const
{
  return pointcloud_org_;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PlanarSegmentation<PointT>::getOriginalTransformedCloud() const
{
  return cloud_org_transformed_;
}

template <class PointT>
pcl::ModelCoefficients::Ptr PlanarSegmentation<PointT>::getPlaneCoeff() const
{
  return plane_coeff_;
}
}

#endif  // SEGMENT_TABLE_H
