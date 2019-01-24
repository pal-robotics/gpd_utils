#ifndef PAL_GRASP_GENERATION_SERVER_H
#define PAL_GRASP_GENERATION_SERVER_H

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
#include <actionlib/server/simple_action_server.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPG
#include <gpg/cloud_camera.h>

// gpd project (messages)
#include <gpd/CloudIndexed.h>
#include <gpd/CloudSamples.h>
#include <gpd/CloudSources.h>
#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>
#include <gpd/SamplesMsg.h>

// this project (messages)
#include <gpd_utils/GraspCandidatesGenerationAction.h>

// this project (headers)
#include <gpd/grasp_detector.h>
#include <gpd/grasp_plotter.h>
#include <gpd/sequential_importance_sampling.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

namespace pal
{
/** PalGraspGenerationServer class
 *
 * \brief A ROS node that can detect grasp poses in a point cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class PalGraspGenerationServer
{
public:
  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  PalGraspGenerationServer(ros::NodeHandle& node);

  /**
   * \brief Destructor.
  */
  ~PalGraspGenerationServer()
  {
  }

  /**
   * \brief Detect grasp poses in a point cloud received from a ROS topic.
   * \return the list of grasp poses
  */
  std::vector<Grasp> detectGraspPosesInPointCloud() const;


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
   * @brief convertToGraspCandidates function that helps to convert the gpd type graps to
   * a vector of grasp candidates
   * @param grasps - input a vector of gpd type grasp
   * @param grasp_cand - outputs a vector of geometry_msgs PoseStamped
   */
  void convertToGraspCandidates(const std::vector<Grasp>& grasps,
                                std::vector<geometry_msgs::PoseStamped>& grasp_cand) const;

  /**
   * @brief generateCandidates a function that deals with ros action server
   * @param goal goal is the input of point cloud and receive the grasp candidates as
   * result
   */
  void generateCandidates(const gpd_utils::GraspCandidatesGenerationGoalConstPtr& goal);

  /**
   * \brief Create a ROS message that contains a list of grasp poses from a list of
   * handles.
   * \param hands the list of grasps
   * \return the ROS message that contains the grasp poses
  */
  gpd::GraspConfigList createGraspListMsg(const std::vector<Grasp>& hands) const;

  gpd::GraspConfig convertToGraspMsg(const Grasp& hand) const;

  Eigen::Vector3d view_point_;  ///< (input) view point of the camera onto the point cloud

  std::unique_ptr<CloudCamera> cloud_camera_;  ///< stores point cloud with (optional)
                                               /// camera information
  /// and surface normals
  std_msgs::Header cloud_camera_header_;  ///< stores header of the point cloud

  int size_left_cloud_;  ///< (input) size of the left point cloud (when using two point
                         /// clouds as input)
  bool has_cloud_, has_normals_,
      has_samples_;              ///< status variables for received (input) messages
  std::string frame_;            ///< point cloud frame
  ros::Subscriber samples_sub_;  ///< ROS subscriber for samples messages
  ros::Publisher grasps_pub_;    ///< ROS publisher for grasp list messages

  bool use_importance_sampling_;   ///< if importance sampling is used
  bool use_rviz_;                  ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_;  ///< workspace limits

  std::unique_ptr<GraspDetector> grasp_detector_;  ///< used to run the GPD algorithm
  std::unique_ptr<SequentialImportanceSampling> importance_sampling_;  ///< sequential
                                                                       /// importance
  /// sampling
  /// variation of GPD algorithm
  std::unique_ptr<GraspPlotter> rviz_plotter_;  ///< used to plot detected grasps in rviz
  ros::NodeHandle nh_;
  ros::Publisher grasp_pose_pub_;

  actionlib::SimpleActionServer<gpd_utils::GraspCandidatesGenerationAction> grasp_generation_server_;
};
}

#endif  // PAL_GRASP_GENERATION_SERVER_H
