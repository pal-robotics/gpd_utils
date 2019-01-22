#ifndef OBJECT_RECOGNITION_INFO_H
#define OBJECT_RECOGNITION_INFO_H

// PAL headers
#include <gpd_utils/pcl_filters.hpp>
#include <pal_detection_msgs/RecognizeObjectsAction.h>
#include <pal_detection_msgs/RecognizedObjectArray.h>
#include <gpd_utils/BoundingBox.h>

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
  ObjectRecognitionInfo(ros::NodeHandle &nh);

  virtual ~ObjectRecognitionInfo();

  bool computeObjectBoundingBox(const std::string &object_name,
                                gpd_utils::BoundingBox &bbox);

  void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud);

  void setImage(sensor_msgs::CompressedImagePtr &image);

private:
  void pixelTo3DPoint(const int &u, const int &v, geometry_msgs::PointStamped &p);

  void recognizedObjectsInfo(const pal_detection_msgs::RecognizedObjectArray &recognized_objects,
                             std::vector<std::string> &classes,
                             std::vector<sensor_msgs::RegionOfInterest> &bboxes);

  void transformPoint(const std::string &frame_id, geometry_msgs::PointStamped &point_out);

  void computeBBoxPoints(int &xmin, int &ymin, int &xmax, int &ymax,
                         gpd_utils::BoundingBox &bbox);

  bool isOverlap(const int &object_index, const std::vector<sensor_msgs::RegionOfInterest> &BBoxes,
                 const std::vector<std::string> &classes);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
  sensor_msgs::CompressedImagePtr image_;

  ros::NodeHandle &nh_;

  tf::TransformListener _tfListener;

  // frame in which the point cloud will be transformed and processed
  std::string _outFrame;

  geometry_msgs::PointStamped point_lu, point_rb, point_ctr;
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
