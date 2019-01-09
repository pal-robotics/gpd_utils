#include <gpd_utils/pal_grasp_generation_server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
using namespace pal;

PalGraspGenerationServer::PalGraspGenerationServer(ros::NodeHandle& node)
  : has_cloud_(false)
  , has_normals_(false)
  , size_left_cloud_(0)
  , has_samples_(true)
  , frame_("")
  , nh_(node)
  , grasp_generation_server_(
        nh_, "/generate_grasp_candidates",
        boost::bind(&PalGraspGenerationServer::generateCandidates, this, _1), false)
{
  cloud_camera_ = NULL;

  // set camera viewpoint to default origin
  std::vector<double> camera_position;
  nh_.getParam("camera_position", camera_position);
  view_point_ << camera_position[0], camera_position[1], camera_position[2];

  // choose sampling method for grasp detection
  nh_.param("use_importance_sampling", use_importance_sampling_, false);

  if (use_importance_sampling_)
  {
    importance_sampling_ = new SequentialImportanceSampling(nh_);
  }
  grasp_detector_ = new GraspDetector(nh_);

  // Read input cloud and sample ROS topics parameters.
  std::string cloud_topic;
  nh_.param("cloud_topic", cloud_topic, std::string("/xtion/depth_registered/points"));
  std::string samples_topic;
  nh_.param("samples_topic", samples_topic, std::string(""));
  std::string rviz_topic;
  nh_.param("rviz_topic", rviz_topic, std::string(""));

  if (!rviz_topic.empty())
  {
    rviz_plotter_ = new GraspPlotter(nh_, grasp_detector_->getHandSearchParameters());
    use_rviz_ = true;
  }
  else
  {
    use_rviz_ = false;
  }

  // subscribe to input samples ROS topic
  if (!samples_topic.empty())
  {
    samples_sub_ =
        nh_.subscribe(samples_topic, 1, &PalGraspGenerationServer::samples_callback, this);
    has_samples_ = false;
  }

  // uses ROS topics to publish grasp candidates, antipodal grasps, and grasps after
  // clustering
  grasps_pub_ = nh_.advertise<gpd::GraspConfigList>("clustered_grasps", 10);

  grasp_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("grasping_pose", 10);

  nh_.getParam("workspace", workspace_);

  grasp_generation_server_.start();
  ROS_INFO_STREAM("Grasp Generation Action Server is Running!!");
}

void PalGraspGenerationServer::convertToGraspPoses(const std::vector<Grasp>& grasps,
                                                   geometry_msgs::PoseArray& grasp_poses)
{
  grasp_poses.header = cloud_camera_header_;
  geometry_msgs::Pose grasp_pose;
  for (size_t i = 0; i < grasps.size(); i++)
  {
    if (std::isnan(grasps.at(i).getScore()))
    {
      ROS_DEBUG("Found a nan score value, skipping the grasp_pose generation");
      continue;
    }
    Eigen::Quaterniond orientation(grasps.at(i).getFrame());
    tf::quaternionEigenToMsg(orientation, grasp_pose.orientation);
    tf::pointEigenToMsg(grasps.at(i).getGraspBottom(), grasp_pose.position);
    grasp_poses.poses.push_back(grasp_pose);
  }
}

void PalGraspGenerationServer::convertToGraspCandidates(
    const std::vector<Grasp>& grasps, std::vector<geometry_msgs::PoseStamped>& grasp_cand)
{
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header = cloud_camera_header_;
  for (size_t i = 0; i < grasps.size(); i++)
  {
    if (std::isnan(grasps.at(i).getScore()))
    {
      ROS_DEBUG("Found a nan score value, skipping the grasp candidate generation");
      continue;
    }
    Eigen::Quaterniond orientation(grasps.at(i).getFrame());
    tf::quaternionEigenToMsg(orientation, grasp_pose.pose.orientation);
    tf::pointEigenToMsg(grasps.at(i).getGraspBottom(), grasp_pose.pose.position);
    grasp_cand.push_back(grasp_pose);
  }
}

std::vector<Grasp> PalGraspGenerationServer::detectGraspPosesInPointCloud()
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

std::vector<int> PalGraspGenerationServer::getSamplesInBall(const PointCloudRGBA::Ptr& cloud,
                                                            const pcl::PointXYZRGBA& centroid,
                                                            float radius)
{
  std::vector<int> indices;
  std::vector<float> dists;
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);
  kdtree.radiusSearch(centroid, radius, indices, dists);
  return indices;
}

void PalGraspGenerationServer::generateCandidates(const gpd_utils::GraspCandidatesGenerationGoalConstPtr& goal)
{
  if ((goal->pointcloud.width * goal->pointcloud.height) != 0)
  {
    delete cloud_camera_;
    cloud_camera_ = NULL;
    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point_;

    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(goal->pointcloud, *cloud);
    cloud_camera_ = new CloudCamera(cloud, 0, view_points);
    cloud_camera_header_ = goal->pointcloud.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size()
                                           << " points.");

    grasp_detector_->setTableHeight(goal->table_height);

    has_cloud_ = true;
    frame_ = goal->pointcloud.header.frame_id;

    std::vector<Grasp> grasps = detectGraspPosesInPointCloud();

    std::vector<geometry_msgs::PoseStamped> grasp_candidates;

    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
      geometry_msgs::PoseArray grasp_candidates_msg;
      this->convertToGraspPoses(grasps, grasp_candidates_msg);
      grasp_pose_pub_.publish(grasp_candidates_msg);
    }

    gpd_utils::GraspCandidatesGenerationResult result;
    this->convertToGraspCandidates(grasps, result.grasp_candidates);
    grasp_generation_server_.setSucceeded(result);
  }
  else
  {
    ROS_WARN_STREAM("Recieved empty pointcloud data, returning zero grasp candidates");
    gpd_utils::GraspCandidatesGenerationResult result;
    grasp_generation_server_.setSucceeded(result);
  }
}

void PalGraspGenerationServer::samples_callback(const gpd::SamplesMsg& msg)
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


void PalGraspGenerationServer::initCloudCamera(const gpd::CloudSources& msg)
{
  // clean up
  delete cloud_camera_;
  cloud_camera_ = NULL;

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

    cloud_camera_ = new CloudCamera(cloud, camera_source, view_points);
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

    cloud_camera_ = new CloudCamera(cloud, camera_source, view_points);
    std::cout << "view_points:\n" << view_points << "\n";
  }
}


gpd::GraspConfigList PalGraspGenerationServer::createGraspListMsg(const std::vector<Grasp>& hands)
{
  gpd::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++)
    msg.grasps.push_back(convertToGraspMsg(hands[i]));

  msg.header = cloud_camera_header_;

  return msg;
}


gpd::GraspConfig PalGraspGenerationServer::convertToGraspMsg(const Grasp& hand)
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
  ros::init(argc, argv, "pal_grasp_generation_server");
  ros::NodeHandle node("~");

  PalGraspGenerationServer grasp_detection(node);
  //  grasp_detection.run();
  ros::spin();

  return 0;
}
