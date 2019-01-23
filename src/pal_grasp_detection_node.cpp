#include <gpd_utils/pal_grasp_detection_node.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
using namespace pal;


/** constants for input point cloud types */
const int PalGraspDetectionNode::POINT_CLOUD_2 = 0;  ///< sensor_msgs/PointCloud2
const int PalGraspDetectionNode::CLOUD_INDEXED = 1;  ///< cloud with indices
const int PalGraspDetectionNode::CLOUD_SAMPLES = 2;  ///< cloud with (x,y,z) samples


PalGraspDetectionNode::PalGraspDetectionNode(ros::NodeHandle& node, std::string target_frame)
  : has_cloud_(false)
  , has_normals_(false)
  , size_left_cloud_(0)
  , has_samples_(true)
  , frame_(target_frame)
  , nh_(node)
{
  cloud_camera_.reset();

  // set camera viewpoint to default origin
  std::vector<double> camera_position;
  node.getParam("camera_position", camera_position);
  view_point_ << camera_position[0], camera_position[1], camera_position[2];

  // choose sampling method for grasp detection
  node.param("use_importance_sampling", use_importance_sampling_, false);

  if (use_importance_sampling_)
  {
    importance_sampling_.reset(new SequentialImportanceSampling(node));
  }
  grasp_detector_.reset(new GraspDetector(node));

  // Read input cloud and sample ROS topics parameters.
  int cloud_type;
  node.param("cloud_type", cloud_type, POINT_CLOUD_2);
  std::string cloud_topic;
  node.param("cloud_topic", cloud_topic, std::string("/xtion/depth_registered/points"));
  std::string samples_topic;
  node.param("samples_topic", samples_topic, std::string(""));
  std::string rviz_topic;
  node.param("rviz_topic", rviz_topic, std::string(""));

  if (!rviz_topic.empty())
  {
    grasps_rviz_pub_ = node.advertise<visualization_msgs::MarkerArray>(rviz_topic, 1);
    rviz_plotter_.reset(new GraspPlotter(node, grasp_detector_->getHandSearchParameters()));
    use_rviz_ = true;
  }
  else
  {
    use_rviz_ = false;
  }

  // subscribe to input point cloud ROS topic
  if (cloud_type == POINT_CLOUD_2)
    cloud_sub_ = node.subscribe(cloud_topic, 1, &PalGraspDetectionNode::cloud_callback, this);
  else if (cloud_type == CLOUD_INDEXED)
    cloud_sub_ =
        node.subscribe(cloud_topic, 1, &PalGraspDetectionNode::cloud_indexed_callback, this);
  else if (cloud_type == CLOUD_SAMPLES)
  {
    cloud_sub_ =
        node.subscribe(cloud_topic, 1, &PalGraspDetectionNode::cloud_samples_callback, this);
    //    grasp_detector_->setUseIncomingSamples(true);
    has_samples_ = false;
  }

  // subscribe to input samples ROS topic
  if (!samples_topic.empty())
  {
    samples_sub_ =
        node.subscribe(samples_topic, 1, &PalGraspDetectionNode::samples_callback, this);
    has_samples_ = false;
  }

  // uses ROS topics to publish grasp candidates, antipodal grasps, and grasps after
  // clustering
  grasps_pub_ = node.advertise<gpd::GraspConfigList>("clustered_grasps", 10);

  pose_pub = nh_.advertise<geometry_msgs::PoseArray>("/marker_pose", 1);

  node.getParam("workspace", workspace_);
}


void PalGraspDetectionNode::run()
{
  ros::Rate rate(100);
  ROS_INFO("Waiting for point cloud to arrive ...");

  while (ros::ok())
  {
    if (has_cloud_)
    {
      // Detect grasps in point cloud.
      std::vector<Grasp> grasps = detectGraspPosesInTopic();
      geometry_msgs::PoseArray grasp_candidates;
      std::vector<Grasp> disp_grasps;
      if (grasps.size() > 0)
      {
        disp_grasps.push_back(grasps.at(0));

        tf::Matrix3x3 rot =
            tf::Matrix3x3(grasps.at(0).getApproach()[0], grasps.at(0).getApproach()[1],
                          grasps.at(0).getApproach()[2], grasps.at(0).getBinormal()[0],
                          grasps.at(0).getBinormal()[1], grasps.at(0).getBinormal()[2],
                          grasps.at(0).getAxis()[0], grasps.at(0).getAxis()[1],
                          grasps.at(0).getAxis()[2]);
        tf::Quaternion qat;
        Eigen::Quaterniond quat(grasps.at(0).getFrame());
        rot.getRotation(qat);

        //      ros::Publisher pose_pub =
        //      nh_.advertise<geometry_msgs::PoseStamped>("/marker_pose", 1);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = cloud_camera_header_;
        tf::quaternionEigenToMsg(quat, pose_stamped.pose.orientation);
        //          tf::pointEigenToMsg((grasps.at(0).getGraspTop()+grasps.at(0).getPosition())*0.5,
        //          pose_stamped.pose.position);
        tf::pointEigenToMsg(grasps.at(0).getGraspBottom(), pose_stamped.pose.position);

        ROS_INFO_STREAM(pose_stamped);
        //          pose_pub.publish(pose_stamped);
      }

      // Visualize the detected grasps in rviz.
      if (use_rviz_)
      {
        rviz_plotter_->drawGrasps(grasps, frame_);
        this->convertToGraspPoses(grasps, grasp_candidates);
        pose_pub.publish(grasp_candidates);
      }

      // Reset the system.
      has_cloud_ = false;
      has_samples_ = false;
      has_normals_ = false;
      ROS_INFO("Waiting for point cloud to arrive ...");
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void PalGraspDetectionNode::convertToGraspPoses(const std::vector<Grasp>& grasps,
                                                geometry_msgs::PoseArray& grasp_poses) const
{
  grasp_poses.header = cloud_camera_header_;
  geometry_msgs::Pose grasp_pose;
  for (size_t i = 0; i < grasps.size(); i++)
  {
    if (std::isnan(grasps.at(i).getScore()))
    {
      ROS_DEBUG("Found a nan score value, skipping the candidate generation");
      continue;
    }
    Eigen::Quaterniond orientation(grasps.at(i).getFrame());
    tf::quaternionEigenToMsg(orientation, grasp_pose.orientation);
    tf::pointEigenToMsg(grasps.at(i).getGraspBottom(), grasp_pose.position);
    grasp_poses.poses.push_back(grasp_pose);
  }
}

std::vector<Grasp> PalGraspDetectionNode::detectGraspPosesInTopic() const
{
  // detect grasp poses
  std::vector<Grasp> grasps;

  if (use_importance_sampling_)
  {
    cloud_camera_->filterWorkspace(workspace_);
    cloud_camera_->voxelizeCloud(0.003);
    cloud_camera_->calculateNormals(4);
    grasps = importance_sampling_->detectGrasps(*cloud_camera_);
  }
  else
  {
    // preprocess the point cloud
    grasp_detector_->preprocessPointCloud(*cloud_camera_);

    // detect grasps in the point cloud
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);
  }

  // Publish the selected grasps.
  gpd::GraspConfigList selected_grasps_msg = createGraspListMsg(grasps);
  grasps_pub_.publish(selected_grasps_msg);
  ROS_INFO_STREAM("Published " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");

  return grasps;
}


std::vector<int> PalGraspDetectionNode::getSamplesInBall(const PointCloudRGBA::Ptr& cloud,
                                                         const pcl::PointXYZRGBA& centroid,
                                                         float radius) const
{
  std::vector<int> indices;
  std::vector<float> dists;
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);
  kdtree.radiusSearch(centroid, radius, indices, dists);
  return indices;
}


void PalGraspDetectionNode::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!has_cloud_)
  {
    cloud_camera_.reset();

    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point_;

    if (msg->fields.size() == 6 && msg->fields[3].name == "normal_x" &&
        msg->fields[4].name == "normal_y" && msg->fields[5].name == "normal_z")
    {
      PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
      pcl::fromROSMsg(*msg, *cloud);
      cloud_camera_.reset(new CloudCamera(cloud, 0, view_points));
      cloud_camera_header_ = msg->header;
      ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size()
                                             << " points and normals.");
    }
    else
    {
      PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
      sensor_msgs::PointCloud2Ptr transformed_msg(new sensor_msgs::PointCloud2);
      pcl_ros::transformPointCloud(frame_, *msg, *transformed_msg, _tfListener);
      pcl::fromROSMsg(*transformed_msg, *cloud);
      cloud_camera_.reset(new CloudCamera(cloud, 0, view_points));
      cloud_camera_header_ = msg->header;
      ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size()
                                             << " points.");
    }

    has_cloud_ = true;
    frame_ = msg->header.frame_id;
  }
}


void PalGraspDetectionNode::cloud_indexed_callback(const gpd::CloudIndexed& msg)
{
  if (!has_cloud_)
  {
    initCloudCamera(msg.cloud_sources);

    // Set the indices at which to sample grasp candidates.
    std::vector<int> indices(msg.indices.size());
    for (int i = 0; i < indices.size(); i++)
    {
      indices[i] = msg.indices[i].data;
    }
    cloud_camera_->setSampleIndices(indices);

    has_cloud_ = true;
    frame_ = msg.cloud_sources.cloud.header.frame_id;

    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
                                           << msg.indices.size() << " samples");
  }
}


void PalGraspDetectionNode::cloud_samples_callback(const gpd::CloudSamples& msg)
{
  if (!has_cloud_)
  {
    initCloudCamera(msg.cloud_sources);

    // Set the samples at which to sample grasp candidates.
    Eigen::Matrix3Xd samples(3, msg.samples.size());
    for (int i = 0; i < msg.samples.size(); i++)
    {
      samples.col(i) << msg.samples[i].x, msg.samples[i].y, msg.samples[i].z;
    }
    cloud_camera_->setSamples(samples);

    has_cloud_ = true;
    has_samples_ = true;
    frame_ = msg.cloud_sources.cloud.header.frame_id;

    ROS_INFO_STREAM("Received cloud with "
                    << cloud_camera_->getCloudProcessed()->size() << " points, and "
                    << cloud_camera_->getSamples().cols() << " samples");
  }
}


void PalGraspDetectionNode::samples_callback(const gpd::SamplesMsg& msg)
{
  if (!has_samples_)
  {
    Eigen::Matrix3Xd samples(3, msg.samples.size());

    for (int i = 0; i < msg.samples.size(); i++)
    {
      samples.col(i) << msg.samples[i].x, msg.samples[i].y, msg.samples[i].z;
    }

    cloud_camera_->setSamples(samples);
    has_samples_ = true;

    ROS_INFO_STREAM("Received grasp samples message with " << msg.samples.size() << " samples");
  }
}


void PalGraspDetectionNode::initCloudCamera(const gpd::CloudSources& msg)
{
  // Set view points.
  Eigen::Matrix3Xd view_points(3, msg.view_points.size());
  for (int i = 0; i < msg.view_points.size(); i++)
  {
    view_points.col(i) << msg.view_points[i].x, msg.view_points[i].y, msg.view_points[i].z;
  }

  // Set point cloud.
  if (msg.cloud.fields.size() == 6 && msg.cloud.fields[3].name == "normal_x" &&
      msg.cloud.fields[4].name == "normal_y" && msg.cloud.fields[5].name == "normal_z")
  {
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(msg.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < msg.camera_source.size(); i++)
    {
      camera_source(msg.camera_source[i].data, i) = 1;
    }

    cloud_camera_.reset(new CloudCamera(cloud, camera_source, view_points));
  }
  else
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(msg.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < msg.camera_source.size(); i++)
    {
      camera_source(msg.camera_source[i].data, i) = 1;
    }

    cloud_camera_.reset(new CloudCamera(cloud, camera_source, view_points));
    std::cout << "view_points:\n" << view_points << "\n";
  }
}


gpd::GraspConfigList PalGraspDetectionNode::createGraspListMsg(const std::vector<Grasp>& hands) const
{
  gpd::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++)
    msg.grasps.push_back(convertToGraspMsg(hands[i]));

  msg.header = cloud_camera_header_;

  return msg;
}


gpd::GraspConfig PalGraspDetectionNode::convertToGraspMsg(const Grasp& hand) const
{
  gpd::GraspConfig msg;
  tf::pointEigenToMsg(hand.getGraspBottom(), msg.bottom);
  tf::pointEigenToMsg(hand.getGraspTop(), msg.top);
  tf::pointEigenToMsg(hand.getGraspSurface(), msg.surface);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  return msg;
}


int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "detect_grasps");
  ros::NodeHandle node("~");

  PalGraspDetectionNode grasp_detection(node, "/base_footprint");
  grasp_detection.run();

  return 0;
}
