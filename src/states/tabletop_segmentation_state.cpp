#include <gpd_utils/states/tabletop_segmentation_state.h>

using namespace pal;

TableTopSegmentationState::TableTopSegmentationState(const ros::NodeHandle &nh,
                                                     const ros::NodeHandle &pnh,
                                                     const PlanarSegmentationParams &params,
                                                     int num_attempts)
  : nh_(nh)
  , pnh_(pnh)
  , segment_plane_(nh_, pnh_, params)
  , num_attempts_(num_attempts)
  , State({ smach_c::SUCCESS, smach_c::PREEMPTED, smach_c::FAILURE })
{
}

TableTopSegmentationState::~TableTopSegmentationState()
{
}

std::string TableTopSegmentationState::execute(smach_c::UserData &user_data)
{
  bool get_data_successful(false);
  for (int i = 0; i < num_attempts_ && !get_data_successful && !preemptRequested(); i++)
    get_data_successful = segment_plane_.performTableSegmentation();

  if (preemptRequested())
  {
    ROS_WARN_STREAM("State preempted, table segmentation interrupted.");
    return smach_c::PREEMPTED;
  }

  if (!get_data_successful)
  {
    ROS_WARN_STREAM("Unable to perform the table segmentation in " << num_attempts_ << " attempts");
    return smach_c::FAILURE;
  }

  // added 1 cm offset to the actual table height to prevent close grasps
  user_data.addOrUpdateProperty("~table_height", segment_plane_.getTableHeight() + 0.01);
  user_data.addOrUpdateProperty("~nonplane_cloud", segment_plane_.getNonPlaneCloud());
  user_data.addOrUpdateProperty("~plane_cloud", segment_plane_.getPlaneCloud());
  user_data.addOrUpdateProperty("~plane_coeff", segment_plane_.getPlaneCoeff());
  user_data.addOrUpdateProperty("~tabletop_cloud", segment_plane_.getTableTopCloud());
  user_data.addOrUpdateProperty("~image_scene", segment_plane_.getImage());
  user_data.addOrUpdateProperty("~original_cloud", segment_plane_.getOriginalCloud());
  user_data.addOrUpdateProperty("~original_cloud_transformed",
                                segment_plane_.getOriginalTransformedCloud());

  return smach_c::SUCCESS;
}
