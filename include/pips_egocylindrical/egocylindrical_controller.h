
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PIPS_EGOCYLINDRICAL_CONTROLLER_H_
#define PIPS_EGOCYLINDRICAL_CONTROLLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>

#include <pips_trajectory_testing/pips_cc_wrapper.h>


namespace pips_egocylindrical
{




/**
 * @ brief 
 *
 * A simple nodelet-based controller intended to avoid obstacles using PIPS.
 */
class PipsEgocylindricalTrajectoryController : public turtlebot_trajectory_testing::TurtlebotObstacleAvoidanceController
{
public:
  PipsEgocylindricalTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
  ~PipsEgocylindricalTrajectoryController(){};

  virtual bool init();
  
  static constexpr const char* DEFAULT_NAME= "EgocylindricalController";
  
protected:
  bool isReady(const std_msgs::Header& header);
  
  void sensorCb(const std_msgs::Header& header);
  
  void generateTrajectories();
  
  virtual void setupTrajectoryTesters();

  
private:
  std::string name_;
  ros::NodeHandle pnh_;
  
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;      
  
  typedef turtlebot_trajectory_testing::TurtlebotObstacleAvoidanceController Super;

};

} //ns pips_egocylindrical

#endif /* PIPS_EGOCYLINDRICAL_CONTROLLER_H_ */

