#ifndef GENERATE_GRASPING_CANDIDATES_STATE_H
#define GENERATE_GRASPING_CANDIDATES_STATE_H

#include <ros/ros.h>
#include <smach_c/state.h>
#include <actionlib/client/simple_action_client.h>
#include <gpd_utils/GraspCandidatesGenerationAction.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pal
{
/**
 * @brief The GenerateGraspingCandidatesState class - Generates grasping candidates based
 * on the input of the pointcloud using gpd library
 */
class GenerateGraspingCandidatesState : public smach_c::State
{
public:
  // Outcomes are: smach_c::SUCCESS when the GenerateGraspingCandidatesState is successful
  //               smach_c::FAILURE in the case of failure
  //               smach_c::PREEMPTED in the case of preemption

  GenerateGraspingCandidatesState(ros::NodeHandle &nh);

  virtual ~GenerateGraspingCandidatesState();

  virtual std::string execute(smach_c::UserData &user_data) override;

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<gpd_utils::GraspCandidatesGenerationAction> ac_;
};
}

#endif  // GENERATE_GRASPING_CANDIDATES_STATE_H
