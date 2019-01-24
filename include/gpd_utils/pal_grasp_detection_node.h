#ifndef PAL_GRASP_DETECTION_NODE_H
#define PAL_GRASP_DETECTION_NODE_H

// system
#include <algorithm>
#include <vector>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPG
#include <gpg/cloud_camera.h>

// this project (messages)
#include <gpd/CloudIndexed.h>
#include <gpd/CloudSamples.h>
#include <gpd/CloudSources.h>
#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>
#include <gpd/SamplesMsg.h>

// GPD project headers
#include <gpd/grasp_detector.h>
#include <gpd/grasp_plotter.h>
#include <gpd/sequential_importance_sampling.h>

#include <tf/transform_listener.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

namespace pal
{
/** PalGraspDetectionNode class
 *
 * \brief A ROS node that can detect grasp poses in a point cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class PalGraspDetectionNode
{
public:
  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  PalGraspDetectionNode(ros::NodeHandle& node, std::string target_frame);

  /**
   * \brief Destructor.
  */
  ~PalGraspDetectionNode()
  {}

  /**
   * \brief Run the ROS node. Loops while waiting for incoming ROS messages.
  */
  void run();

  /**
   * \brief Detect grasp poses in a point cloud received from a ROS topic.
   * \return the list of grasp poses
  */
  std::vector<Grasp> detectGraspPosesInTopic() const;


private:
  /**
   * \brief Find the indices of the points within a ball around a given point in the
   * cloud.
   * \param cloud the point cloud
   * \param centroid the centroid of the ball
   * \param radius the radius of the ball
   * \return the indices of the points in the point cloud that lie within the ball
  */
  std::vector<int> getSamplesInBall(const PointCloudRGBA::Ptr& cloud,
                                    const pcl::PointXYZRGBA& centroid, float radius) const;

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud.
   * \param msg the incoming ROS message
  */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud and a
   * list of indices.
   * \param msg the incoming ROS message
  */
  void cloudIndexedCallback(const gpd::CloudIndexed& msg);

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud and a
   * list of (x,y,z) samples.
   * \param msg the incoming ROS message
  */
  void cloudSamplesCallback(const gpd::CloudSamples& msg);

  /**
   * \brief Initialize the <cloud_camera> object given a <cloud_sources> message.
   * \param msg the <cloud_sources> message
   */
  void initCloudCamera(const gpd::CloudSources& msg);

  /**
   * \brief Callback function for the ROS topic that contains the input samples.
   * \param msg the incoming ROS message
  */
  void samplesCallback(const gpd::SamplesMsg& msg);

  /**
   * @brief convertToGraspPoses function helps to convert the gpd grasps to normal
   * geometry grasp poses
   * @param grasps - input of the gpd type vector of graps
   * @param grasp_poses - output of poses in geometry_msgs PoseArray
   */
  void convertToGraspPoses(const std::vector<Grasp>& grasps, geometry_msgs::PoseArray& grasp_poses) const;

  /**
   * \brief Create a ROS message that contains a list of grasp poses from a list of
   * handles.
   * \param hands the list of grasps
   * \return the ROS message that contains the grasp poses
  */
  gpd::GraspConfigList createGraspListMsg(const std::vector<Grasp>& hands) const;

  gpd::GraspConfig convertToGraspMsg(const Grasp& hand) const;

  Eigen::Vector3d view_point_;  ///< (input) view point of the camera onto the point cloud

  std::unique_ptr<CloudCamera> cloud_camera_;  ///< stores point cloud with (optional) camera information
                               /// and surface normals
  std_msgs::Header cloud_camera_header_;  ///< stores header of the point cloud

  int size_left_cloud_;  ///< (input) size of the left point cloud (when using two point
                         /// clouds as input)
  bool has_cloud_, has_normals_,
      has_samples_;                 ///< status variables for received (input) messages
  std::string frame_;               ///< point cloud frame
  ros::Subscriber cloud_sub_;       ///< ROS subscriber for point cloud messages
  ros::Subscriber samples_sub_;     ///< ROS subscriber for samples messages
  ros::Publisher grasps_pub_;       ///< ROS publisher for grasp list messages
  ros::Publisher grasps_rviz_pub_;  ///< ROS publisher for grasps in rviz (visualization)

  bool use_importance_sampling_;   ///< if importance sampling is used
  bool use_rviz_;                  ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_;  ///< workspace limits

  std::unique_ptr<GraspDetector> grasp_detector_;                      ///< used to run the GPD algorithm
  std::unique_ptr<SequentialImportanceSampling> importance_sampling_;  ///< sequential importance sampling
                                                       /// variation of GPD algorithm
  std::unique_ptr<GraspPlotter> rviz_plotter_;  ///< used to plot detected grasps in rviz
  ros::NodeHandle nh_;
  ros::Publisher pose_pub;

  tf::TransformListener _tfListener;

  /** constants for input point cloud types */
  static const int POINT_CLOUD_2;  ///< sensor_msgs/PointCloud2
  static const int CLOUD_INDEXED;  ///< gpd/CloudIndexed
  static const int CLOUD_SAMPLES;  ///< gpd/CloudSamples
};
}

#endif  // PAL_GRASP_DETECTION_NODE_H
