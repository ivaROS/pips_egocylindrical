
#include <pips_egocylindrical/hybrid_controller.h>

#include <pips_trajectory_testing/depth_image_cc_wrapper.h>

#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <pips_egocylindrical/multi_cc_wrapper.h>

namespace pips_egocylindrical
{
 
  HybridTrajectoryController::HybridTrajectoryController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    Super(nh, pnh), nh_(nh), pnh_(pnh)
  {

      
  }

  
  
  void HybridTrajectoryController::setupTrajectoryTesters()
  {
        traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh_, ObstacleAvoidanceController::pnh_, "future_trajectory_tester");
        traj_tester_->init();
        
        traj_tester2_ = std::make_shared<TurtlebotGenAndTest>(nh_, ObstacleAvoidanceController::pnh_, "current_trajectory_tester");
        traj_tester2_->init();

        std::shared_ptr<pips_egocylindrical::EgocylindricalRangeImageCCWrapper> ec_cc = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh_, pnh_, tfBuffer_, "future_trajectory_collision_checker");
        std::shared_ptr<pips_trajectory_testing::DepthImageCCWrapper> depth_cc = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh_, pnh_, tfBuffer_, "current_trajectory_collision_checker");
        
        std::shared_ptr<pips_egocylindrical::DepthEgoCCWrapper> combined_cc = std::make_shared<pips_egocylindrical::DepthEgoCCWrapper>(nh_, pnh_, tfBuffer_);
        combined_cc->setCC(depth_cc, ec_cc);
        
        //cc_wrapper1_ = combined_cc;
        cc_wrapper1_ = ec_cc;
        
        cc_wrapper2_ = depth_cc;
        
        
        traj_tester_->setCollisionChecker(cc_wrapper1_->getCC(), cc_wrapper2_->getCC());
        //traj_tester_->setCollisionChecker(cc_wrapper1_->getCC());
        
        traj_tester2_->setCollisionChecker(cc_wrapper2_->getCC());
        
  }
  
     



  // Note: I haven't fully thought through other implementations, but this may be generic after all...
  // If so, then this code will probably move back to the main controller but be renamed 'transformReady' or something
  bool HybridTrajectoryController::isReady(const std_msgs::Header& header)
  {
    bool ready1, ready2;
    
    ros::WallTime t1 = ros::WallTime::now();
    
    #pragma omp parallel num_threads(2) shared(ready1,ready2)
    {
      #pragma omp sections 
      {
        #pragma omp section
        {
          ready1 = cc_wrapper1_->isReady(header);
          if(ready1)
          {
              cc_wrapper1_->update();
          }
      
        }
      
        #pragma omp section
        {
          ready2 = cc_wrapper2_->isReady(header);
          if(ready2)
          {
              cc_wrapper2_->update();
          }
        
        }
        
      }
    
    }
    
    auto t2 = ros::WallTime::now();
    setup_durations_.addDuration(t1,t2);
    
    ROS_DEBUG_STREAM_NAMED(name_ + ".setup_timing", "[" + name_ + "]: Setup Duration = " << setup_durations_.getLastDuration() << ", size=" << -1 << ", average setup duration = " << setup_durations_.averageDuration() );
      
    return ready1 && ready2;
  }
  
  void HybridTrajectoryController::generateTrajectories()
  {
      std_msgs::Header header = cc_wrapper2_->getCurrentHeader();
      Super::sensorCb(header);
  }
  
  bool HybridTrajectoryController::init()
  {
    Super::init();
    
    cc_wrapper1_->init();
    cc_wrapper2_->init();

    cc_wrapper2_->setCallback(boost::bind(&HybridTrajectoryController::generateTrajectories, this));
    
    return true;
  }
  
}
