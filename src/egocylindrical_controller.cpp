#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>

#include <pips_egocylindrical/egocylindrical_controller.h>

namespace pips_egocylindrical
{
 
  PipsEgocylindricalTrajectoryController::PipsEgocylindricalTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    ObstacleAvoidanceController(nh, pnh), nh_(nh), pnh_(pnh)
  {

      
  }

  
  
  void PipsEgocylindricalTrajectoryController::setupTrajectoryTesters()
  {
        traj_tester_ = std::make_shared<GenAndTest>(nh_, pnh_);
        traj_tester_->init();
        traj_tester2_ = traj_tester_;
        
        cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh_, pnh_,tfBuffer_);
        traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
  }
  
     



  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsEgocylindricalTrajectoryController::isReady(const std_msgs::Header& header)
  {
    if(!ObstacleAvoidanceController::isReady(header))
    {
      return false;
    }

    else
    {
        bool ready = cc_wrapper_->isReady(header);
        if(ready)
        {
            cc_wrapper_->update();
        }
      return ready;
    }
  }
  
  void PipsEgocylindricalTrajectoryController::generateTrajectories()
  {
      std_msgs::Header header = cc_wrapper_->getCurrentHeader();
      ObstacleAvoidanceController::sensorCb(header);
  }
  
  bool PipsEgocylindricalTrajectoryController::init()
  {
    ObstacleAvoidanceController::init();
    
    cc_wrapper_->init();
    cc_wrapper_->setCallback(boost::bind(&PipsEgocylindricalTrajectoryController::generateTrajectories, this));
    
    return true;
  }
  
}
