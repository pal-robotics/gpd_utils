#include <gpd_utils/segment_table.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "segment_table");

  ros::NodeHandle nh, pnh("~");

  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))  // NOTE: Important when using
                                                          // simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  pal::PlanarSegmentationParams params;
  params.readConfig<ariles::ros>(nh, "/PlanarSegmentationParams");
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  pal::PlanarSegmentation<pcl::PointXYZRGB> locator(nh, pnh, params);

  while (ros::ok())
  {
    locator.performTableSegmentation();
  }

  return 0;
}
