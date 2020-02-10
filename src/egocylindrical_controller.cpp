#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>

#include <pips_egocylindrical/egocylindrical_controller.h>

using namespace turtlebot_trajectory_testing;

namespace pips_egocylindrical
{
 
  PipsEgocylindricalTrajectoryController::PipsEgocylindricalTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
    Super(nh, pnh, "obstacle_avoidance"),
    name_(name),
    pnh_(pnh)
  {

      
  }

  
  
  void PipsEgocylindricalTrajectoryController::setupTrajectoryTesters()
  {
    traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh_, Super::pnh_);
    traj_tester_->init();
    traj_tester2_ = traj_tester_;
        
    cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh_, pnh_, tfm_);
    traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
}
  
     



  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool PipsEgocylindricalTrajectoryController::isReady(const std_msgs::Header& header)
  {
    if(cc_wrapper_->isReady(header))
    {
      cc_wrapper_->update();
      return true;
    }
    else
    {
      return false;
    }
  }
  
  void PipsEgocylindricalTrajectoryController::generateTrajectories()
  {
      std_msgs::Header header = cc_wrapper_->getCurrentHeader();
      Super::sensorCb(header);
  }
  
  bool PipsEgocylindricalTrajectoryController::init()
  {
    Super::init();
    
    cc_wrapper_->init();
    cc_wrapper_->setCallback(boost::bind(&PipsEgocylindricalTrajectoryController::generateTrajectories, this));
    
    return true;
  }
  
}
