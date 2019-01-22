#ifndef SEGMENT_TABLE_H
#define SEGMENT_TABLE_H

// PAL headers
#include <gpd_utils/pcl_filters.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL headers
#include <pcl/point_cloud.h>

// Std C++ headers
#include <string>

#include <ariles/adapters_all.h>
#include <ariles/ariles_all.h>
#include <ariles/ariles.h>

namespace pal
{
struct PlanarSegmentationParams : public ariles::ConfigurableBase
{
  PlanarSegmentationParams() = default;

  void setDefaults() override
  {
  }
#define ARILES_SECTION_ID "PlanarSegmentationParams"
#define ARILES_CONSTRUCTOR PlanarSegmentationParams
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(rate)                                                                    \
  ARILES_ENTRY_(processing_frame)                                                        \
  ARILES_ENTRY_(passthrough_zmin)                                                        \
  ARILES_ENTRY_(passthrough_zmax)                                                        \
  ARILES_ENTRY_(passthrough_xmin)                                                        \
  ARILES_ENTRY_(passthrough_xmax)                                                        \
  ARILES_ENTRY_(downsampling_size)
#include ARILES_INITIALIZE

  double rate_;
  std::string processing_frame_;
  double passthrough_zmin_;
  double passthrough_zmax_;
  double passthrough_xmin_;
  double passthrough_xmax_;
  double downsampling_size_;
};

template <class PointT>
class PlanarSegmentation
{
public:
  PlanarSegmentation(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                     const PlanarSegmentationParams& params);

  virtual ~PlanarSegmentation();

  bool performTableSegmentation();

  void getOriginalCloud(typename pcl::PointCloud<PointT>::Ptr& original_cloud);

  void getImage(sensor_msgs::CompressedImagePtr& image);

  void getTableTopCloud(typename pcl::PointCloud<PointT>::Ptr& tabletop_cloud);

  void getPlaneCloud(typename pcl::PointCloud<PointT>::Ptr& plane_cloud);

  void getNonPlaneCloud(typename pcl::PointCloud<PointT>::Ptr& nonplane_cloud);

  void getPlaneCoeff(pcl::ModelCoefficients::Ptr& plane_coeff);

  double getTableHeight();

protected:
  void cloudImageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                          const sensor_msgs::CompressedImageConstPtr& image);

  bool processCloudData();

  void getCloudData();

  void publish(typename pcl::PointCloud<PointT>::Ptr& planeCloud,
               typename pcl::PointCloud<PointT>::Ptr& nonPlaneCloud,
               typename pcl::PointCloud<PointT>::Ptr& tableTopCloud,
               pcl::ModelCoefficients::Ptr& planeCoeff, pcl::uint64_t& stamp,
               const std::string& frameId);

  void publishEmptyClouds(pcl::uint64_t& stamp, const std::string& frameId);

  ros::NodeHandle &_nh, _pnh;
  ros::CallbackQueue _cbQueue;

  tf::TransformListener _tfListener;

  // planar segmentation parameters
  PlanarSegmentationParams params_;

  // height of the table
  double table_height_;

  // ROS publishers
  ros::Publisher _planeCloudPub;
  ros::Publisher _nonPlaneCloudPub;
  ros::Publisher _tableTopCloudPub;
  ros::Publisher _planeCoeffPub;

  message_filters::Subscriber<sensor_msgs::PointCloud2> _cloudSub;
  message_filters::Subscriber<sensor_msgs::CompressedImage> _imageSub;
  typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> msg_filters_type;
  boost::scoped_ptr<msg_filters_type> _image_cloud_sync;

  bool has_cloud_;
  //    sensor_msgs::PointCloud2Ptr pointcloud_org_;
  typename pcl::PointCloud<PointT>::Ptr pointcloud_org_;
  typename pcl::PointCloud<PointT>::Ptr pclFilteredPlaneCloud_;
  typename pcl::PointCloud<PointT>::Ptr pclFilteredNonPlaneCloud_;
  typename pcl::PointCloud<PointT>::Ptr pclFilteredTableTopCloud_;
  pcl::ModelCoefficients::Ptr planeCoeff_;
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

  _planeCloudPub = _pnh.advertise<typename pcl::PointCloud<PointT> >("plane", 1, true);
  _nonPlaneCloudPub = _pnh.advertise<typename pcl::PointCloud<PointT> >("nonplane", 1, true);
  _tableTopCloudPub =
      _pnh.advertise<typename pcl::PointCloud<PointT> >("tabletop_cloud", 1, true);
  _planeCoeffPub = _pnh.advertise<pcl_msgs::ModelCoefficients>("plane_coeff", 1, true);

  _image_cloud_sync.reset(new msg_filters_type(_cloudSub, _imageSub, 10));
  _image_cloud_sync->registerCallback(
      boost::bind(&PlanarSegmentation<PointT>::cloudImageCallback, this, _1, _2));
}

template <class PointT>
PlanarSegmentation<PointT>::~PlanarSegmentation()
{
}

template <class PointT>
void PlanarSegmentation<PointT>::publishEmptyClouds(pcl::uint64_t& stamp, const std::string& frameId)
{
  typename pcl::PointCloud<PointT>::Ptr emptyCloud(new typename pcl::PointCloud<PointT>);
  pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
  publish(emptyCloud, emptyCloud, emptyCloud, planeCoeff, stamp, frameId);
}

template <class PointT>
bool PlanarSegmentation<PointT>::performTableSegmentation()
{
  this->getCloudData();
  return this->processCloudData();
}

template <class PointT>
void PlanarSegmentation<PointT>::getCloudData()
{
  ros::Rate loopRate(params_.rate_);
  double halfPeriod = 0.5 * 1.0 / params_.rate_;
  _cloudSub.subscribe(_nh, "cloud_topic", 1);
  _imageSub.subscribe(_nh, "image_topic", 1);
  while (!has_cloud_)
  {
    // check for subscriber's callbacks
    _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

    loopRate.sleep();
  }
  _cloudSub.unsubscribe();
  _imageSub.unsubscribe();
  has_cloud_ = false;
}

template <class PointT>
bool PlanarSegmentation<PointT>::processCloudData()
{
  typename pcl::PointCloud<PointT>::Ptr cloudInProcFrame;

  // Transform the point cloud to the frame specified if any
  if (!params_.processing_frame_.empty())
  {
    cloudInProcFrame.reset(new typename pcl::PointCloud<PointT>);
    ROS_INFO_STREAM("Transforming point cloud from frame "
                    << pointcloud_org_->header.frame_id << " to frame "
                    << params_.processing_frame_);
    //        bool transform_complete =
    //        pcl_ros::transformPointCloud(params_.processing_frame_,
    //        *pointcloud_org_, *cloudInProcFrame, _tfListener);
    bool transform_complete = pcl_ros::transformPointCloud(
        params_.processing_frame_, ros::Time(0), *pointcloud_org_,
        pointcloud_org_->header.frame_id, *cloudInProcFrame, _tfListener);
    if (!transform_complete)
    {
      publishEmptyClouds(pointcloud_org_->header.stamp, params_.processing_frame_);
      return false;
    }
    cloudInProcFrame->header.frame_id = params_.processing_frame_;
  }
  else
    cloudInProcFrame = pointcloud_org_;
  // cloudInProcFrame = boost::make_shared<typename pcl::PointCloud<PointT>
  // >(*pointcloud_org_);
  // cloudInProcFrame.reset(new typename pcl::PointCloud<PointT>(*pointcloud_org_));

  // Apply passthrough filter
  typename pcl::PointCloud<PointT>::Ptr zpassThroughCloud(new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr passThroughCloud(new typename pcl::PointCloud<PointT>);
  std::string axis;
  axis = "z";
  pal::passThrough<PointT>(cloudInProcFrame, axis, params_.passthrough_zmin_,
                           params_.passthrough_zmax_, zpassThroughCloud);

  if (zpassThroughCloud->empty())
  {
    // if all points get removed after the pass-through filtering just return false and
    // stop processing
    publishEmptyClouds(cloudInProcFrame->header.stamp, cloudInProcFrame->header.frame_id);
    return false;
  }

  axis = "x";
  pal::passThrough<PointT>(zpassThroughCloud, axis, params_.passthrough_xmin_,
                           params_.passthrough_xmax_, passThroughCloud);

  if (passThroughCloud->empty())
  {
    // if all points get removed after the pass-through filtering just return false and
    // stop processing
    publishEmptyClouds(cloudInProcFrame->header.stamp, cloudInProcFrame->header.frame_id);
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
    publishEmptyClouds(cloudInProcFrame->header.stamp, cloudInProcFrame->header.frame_id);
    return false;
  }

  // Remove main plane
  typename pcl::PointCloud<PointT>::Ptr pclPlaneCloud(new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr pclNonPlaneCloud(new typename pcl::PointCloud<PointT>);
  planeCoeff_.reset(new pcl::ModelCoefficients);
  pal::planeSegmentation<PointT>(pclDownSampledCloud, &pclPlaneCloud, &pclNonPlaneCloud,
                                 &planeCoeff_);


  // filter outliers in the plane cloud
  pclFilteredPlaneCloud_.reset(new typename pcl::PointCloud<PointT>);
  if (pclPlaneCloud->empty())
    pclFilteredPlaneCloud_ = pclPlaneCloud;
  else
    pal::statisticalOutlierRemoval<PointT>(pclPlaneCloud, 25, 1.0, pclFilteredPlaneCloud_);

  // filter outliers in the cloud not belonging to the main plane
  pclFilteredNonPlaneCloud_.reset(new typename pcl::PointCloud<PointT>);
  if (pclNonPlaneCloud->empty())
    pclFilteredNonPlaneCloud_ = pclNonPlaneCloud;
  else
    pal::statisticalOutlierRemoval<PointT>(pclNonPlaneCloud, 25, 1.0, pclFilteredNonPlaneCloud_);

  // Compute the height of the table respect the ground
  PointT minPt, maxPt;
  pcl::getMinMax3D(*pclFilteredPlaneCloud_, minPt, maxPt);
  planeCoeff_->values.push_back(minPt.z);
  planeCoeff_->values.push_back(maxPt.z);
  table_height_ = maxPt.z;
  // Apply passthrough filter to non plane cloud
  typename pcl::PointCloud<PointT>::Ptr xpassThroughCloud_non_plane(
      new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr ypassThroughCloud_non_plane(
      new typename pcl::PointCloud<PointT>);
  axis = "x";
  pal::passThrough<PointT>(pclFilteredNonPlaneCloud_, axis, minPt.x, maxPt.x,
                           xpassThroughCloud_non_plane);

  axis = "y";
  pal::passThrough<PointT>(xpassThroughCloud_non_plane, axis, minPt.y, maxPt.y,
                           ypassThroughCloud_non_plane);

  pclFilteredNonPlaneCloud_ = ypassThroughCloud_non_plane;

  pclFilteredTableTopCloud_.reset(new typename pcl::PointCloud<PointT>);
  axis = "z";
  pal::passThrough<PointT>(ypassThroughCloud_non_plane, axis, maxPt.z, INFINITY,
                           pclFilteredTableTopCloud_);

  if (pclFilteredTableTopCloud_->empty())
  {
    ROS_WARN_STREAM("The tabletop cloud seems to be empty!!");
    return false;
  }

  ROS_INFO_STREAM("Processing:");
  ROS_INFO_STREAM("\tInput cloud:                 " << cloudInProcFrame->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tAfter pass-through:          " << passThroughCloud->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tAfter downsmapling:          " << pclDownSampledCloud->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tPoints in plane:             " << pclPlaneCloud->points.size() << " points");
  ROS_INFO_STREAM("\tNon-plane points:            " << pclNonPlaneCloud->points.size()
                                                    << " points");
  ROS_INFO_STREAM("\tOutliers in plane:           "
                  << pclPlaneCloud->points.size() - pclFilteredPlaneCloud_->points.size()
                  << " points");
  ROS_INFO_STREAM("\tOutliers in non-plane:       "
                  << pclNonPlaneCloud->points.size() - pclFilteredNonPlaneCloud_->points.size()
                  << " points");
  ROS_INFO_STREAM("\tPlane height.                Min: " << minPt.z << " ,max: " << maxPt.z);
  ROS_INFO_STREAM("\tPlane x.                     Min x: " << minPt.x
                                                           << " ,max x: " << maxPt.x);
  ROS_INFO_STREAM("\tPlane y.                     Min y: " << minPt.y
                                                           << " ,max y: " << maxPt.y);

  publish(pclFilteredPlaneCloud_, pclFilteredNonPlaneCloud_, pclFilteredTableTopCloud_,
          planeCoeff_, cloudInProcFrame->header.stamp, cloudInProcFrame->header.frame_id);

  return true;
}

template <class PointT>
void PlanarSegmentation<PointT>::cloudImageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
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
void PlanarSegmentation<PointT>::publish(typename pcl::PointCloud<PointT>::Ptr& planeCloud,
                                         typename pcl::PointCloud<PointT>::Ptr& nonPlaneCloud,
                                         typename pcl::PointCloud<PointT>::Ptr& tableTopCloud,
                                         pcl::ModelCoefficients::Ptr& planeCoeff,
                                         pcl::uint64_t& stamp, const std::string& frameId)
{
  planeCloud->header.stamp = stamp;
  planeCloud->header.frame_id = frameId;
  _planeCloudPub.publish(planeCloud);

  nonPlaneCloud->header.stamp = stamp;
  nonPlaneCloud->header.frame_id = frameId;
  _nonPlaneCloudPub.publish(nonPlaneCloud);

  tableTopCloud->header.stamp = stamp;
  tableTopCloud->header.frame_id = frameId;
  _tableTopCloudPub.publish(tableTopCloud);

  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*planeCoeff, ros_coefficients);
  pcl_conversions::fromPCL(planeCloud->header, ros_coefficients.header);
  _planeCoeffPub.publish(ros_coefficients);
}

template <class PointT>
void PlanarSegmentation<PointT>::getImage(sensor_msgs::CompressedImagePtr& image)
{
  image = image_org_;
}

template <class PointT>
double PlanarSegmentation<PointT>::getTableHeight()
{
  return table_height_;
}

template <class PointT>
void PlanarSegmentation<PointT>::getNonPlaneCloud(typename pcl::PointCloud<PointT>::Ptr& nonplane_cloud)
{
  nonplane_cloud = pclFilteredNonPlaneCloud_;
}

template <class PointT>
void PlanarSegmentation<PointT>::getPlaneCloud(typename pcl::PointCloud<PointT>::Ptr& plane_cloud)
{
  plane_cloud = pclFilteredPlaneCloud_;
}

template <class PointT>
void PlanarSegmentation<PointT>::getTableTopCloud(typename pcl::PointCloud<PointT>::Ptr& tabletop_cloud)
{
  tabletop_cloud = pclFilteredTableTopCloud_;
}

template <class PointT>
void PlanarSegmentation<PointT>::getOriginalCloud(typename pcl::PointCloud<PointT>::Ptr& original_cloud)
{
  original_cloud = pointcloud_org_;
}

template <class PointT>
void PlanarSegmentation<PointT>::getPlaneCoeff(pcl::ModelCoefficients::Ptr& plane_coeff)
{
  plane_coeff = planeCoeff_;
}
}

#endif  // SEGMENT_TABLE_H
