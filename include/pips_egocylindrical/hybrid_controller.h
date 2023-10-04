
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HYBRID_EGOCYLINDRICAL_CONTROLLER_H_
#define HYBRID_EGOCYLINDRICAL_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>

#include <pips_trajectory_testing/pips_cc_wrapper.h>

#include <memory>


namespace pips_egocylindrical
{




/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class HybridTrajectoryController : public turtlebot_trajectory_testing::TurtlebotObstacleAvoidanceController
{
public:
  typedef turtlebot_trajectory_testing::TurtlebotObstacleAvoidanceController Super;
  
  HybridTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~HybridTrajectoryController(){};

  virtual bool init();

protected:
  bool isReady(const std_msgs::Header& header);
  
  void sensorCb(const std_msgs::Header& header);
  
  void generateTrajectories();
  
  virtual void setupTrajectoryTesters();

  
private:
  std::string name_ = "hybrid_controller";
  ros::NodeHandle nh_, pnh_;
  
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper1_, cc_wrapper2_;      
  pips::utils::DurationAccumulator setup_durations_;
  

};

} //ns pips_egocylindrical

#endif /* HYBRID_EGOCYLINDRICAL_CONTROLLER_H_ */

