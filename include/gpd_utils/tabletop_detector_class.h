#ifndef TABLETOP_DETECTOR_CLASS_H
#define TABLETOP_DETECTOR_CLASS_H

// PAL headers
#include <pal_pcl/pcl_filters.hpp>
#include <gpd_utils/object_bounding_box.h>

// ROS headers
#include <ros/ros.h>

// PCL headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pal
{
template <class PointT>
class TableTopDetector
{
public:
  TableTopDetector(ros::NodeHandle &nh);

  virtual ~TableTopDetector();

  bool extractOneCluster(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                         typename pcl::PointCloud<PointT>::Ptr &clusterCloud,
                         int points_threshold = 100) const;

  void getObjectCloud(const typename pcl::PointCloud<PointT>::Ptr &tabletop_cloud,
                      const gpd_utils::BoundingBox &bb_points,
                      typename pcl::PointCloud<PointT>::Ptr &object_cloud) const;

private:
  ros::NodeHandle nh_;
  bool enabled_;
  double rate_;

  // ROS rate and time
  boost::scoped_ptr<ros::Rate> loop_rate_;
  double period_;

  typename pcl::PointCloud<PointT>::Ptr diff_cloud_;
};

template <class PointT>
TableTopDetector<PointT>::TableTopDetector(ros::NodeHandle &nh)
  : nh_(nh), enabled_(false), rate_(1.0)
{
  diff_cloud_.reset(new pcl::PointCloud<PointT>());

  ROS_INFO_STREAM("The node will operate at " << rate_ << " Hz");

  loop_rate_.reset(new ros::Rate(rate_));
  period_ = 1.0 / rate_;
}

template <class PointT>
TableTopDetector<PointT>::~TableTopDetector()
{
  diff_cloud_.reset();
}

template <class PointT>
bool TableTopDetector<PointT>::extractOneCluster(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                                 typename pcl::PointCloud<PointT>::Ptr &clusterCloud,
                                                 int points_threshold) const
{
  if ((cloud->width * cloud->height) == 0)
  {
    ROS_DEBUG("Empty input point cloud!!");
    return false;
  }

  // Euclidean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices = {};
  pal::euclideanClusterExtraction<PointT>(cloud, cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header = cloud->header;

    if (cloud_cluster->points.size() < points_threshold)
    {
      ROS_INFO_STREAM("Found a cluster with less than "
                      << points_threshold << " points, skipping the cluster");
      continue;
    }
    else
    {
      ROS_INFO_STREAM("Found pointcloud representing the cluster: "
                      << cloud_cluster->points.size() << " data points.");
      pcl::copyPointCloud(*cloud_cluster, *clusterCloud);
      return true;
    }
  }
  return true;
}

template <class PointT>
void TableTopDetector<PointT>::getObjectCloud(const typename pcl::PointCloud<PointT>::Ptr &tabletop_cloud,
                                              const gpd_utils::BoundingBox &bb_points,
                                              typename pcl::PointCloud<PointT>::Ptr &object_cloud) const
{
  double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
  xmin = 0.7 * bb_points.ctr_point_.point.x;
  xmax = 1.2 * bb_points.ctr_point_.point.x;
  ymin = bb_points.rb_point_.point.y;
  ymax = bb_points.lu_point_.point.y;

  /// Checks if the ymax is less than the ymin, which usually happens when the objects
  /// is covered by background, and in some cases, when the ymax is less than the
  /// objects centre point, and When the ymax is too far compared than it should be,
  /// then it tries to define its own limit computing from the center point and the
  /// minimum point
  ymax = ((ymax < ymin) || (ymax < bb_points.ctr_point_.point.y) ||
          (ymax > (bb_points.ctr_point_.point.y + 2.0 * (bb_points.ctr_point_.point.y - ymin)))) ?
             (bb_points.ctr_point_.point.y + (bb_points.ctr_point_.point.y - ymin)) :
             ymax;

  ROS_INFO("Received point cloud with %ld points", tabletop_cloud->points.size());

  pal::passThrough<PointT>(tabletop_cloud, "y", ymin, ymax, object_cloud);
  pal::passThrough<PointT>(object_cloud, "x", xmin, xmax, object_cloud);
  ROS_INFO("Found objects point cloud of %ld points", object_cloud->size());


  //    /// Finding the non_object point cloud
  //    passThroughCloud.reset(new pcl::PointCloud<PointT>());
  //    pal::passThrough<PointT>(diff_cloud_, "z", zmin, zmax, passThroughCloud);
  //    pal::passThrough<PointT>(passThroughCloud, "y", ymin, ymax, passThroughCloud,
  //    true);
  //    ROS_INFO("Leftover point cloud has %ld points", passThroughCloud->size());
  //    diff_cloud_.reset(new pcl::PointCloud<PointT>());
  //    pcl::copyPointCloud(*passThroughCloud, *diff_cloud_);
}
}

#endif  // TABLETOP_DETECTOR_CLASS_H
