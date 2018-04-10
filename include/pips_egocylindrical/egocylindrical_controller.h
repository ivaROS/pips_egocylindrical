
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

#include <pips_egocylindrical/egocylindrical_image_collision_checker.h>

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
  virtual bool isReady(const std_msgs::Header& header);

  virtual void ecImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
                         const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg);

private:
  std::string name_ = "EgocylindricalController";
  ros::NodeHandle nh_, pnh_;
  std::shared_ptr<pips::collision_testing::EgocylindricalImageCollisionChecker> cc_;

  

  
  bool hasTransform_ = false;
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                          egocylindrical::EgoCylinderPoints> image_sync_policy;
  typedef message_filters::Synchronizer<image_sync_policy> image_synchronizer;
  
  message_filters::Subscriber<sensor_msgs::Image> ec_sub_;
  message_filters::Subscriber<egocylindrical::EgoCylinderPoints> ec_info_sub_;
  boost::shared_ptr<image_synchronizer> synced_images;
 

               
               


};

} //ns pips_egocylindrical

#endif /* PIPS_EGOCYLINDRICAL_CONTROLLER_H_ */

