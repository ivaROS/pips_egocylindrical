
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PIPS_EGOCYLINDRICAL_CONTROLLER_H_
#define PIPS_EGOCYLINDRICAL_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "obstacle_avoidance_controller.h"
#include "pips_trajectory_tester.h"

#include <pips_trajectory_testing/pips_cc_wrapper.h>

#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>

#include <ros/callback_queue.h>


namespace pips_egocylindrical
{




/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class PipsEgocylindricalTrajectoryController : public kobuki::ObstacleAvoidanceController
{
public:
  PipsEgocylindricalTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~PipsEgocylindricalTrajectoryController(){};

  virtual bool init();

protected:
  bool isReady(const std_msgs::Header& header);
  
  void sensorCb(const std_msgs::Header& header);
  
  void generateTrajectories();
  
  virtual void setupTrajectoryTesters();

  
private:
  std::string name_ = "EgocylindricalController";
  ros::NodeHandle nh_, pnh_;
  
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;      


};

} //ns pips_egocylindrical

#endif /* PIPS_EGOCYLINDRICAL_CONTROLLER_H_ */

