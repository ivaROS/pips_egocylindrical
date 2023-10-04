#ifndef MULTI_CC_WRAPPER_H
#define MULTI_CC_WRAPPER_H

#include <pips_trajectory_testing/depth_image_cc_wrapper.h>

#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>

namespace pips_egocylindrical
{

class DepthEgoCollisionChecker : public pips::collision_testing::TransformingCollisionChecker //, private pips::collision_testing::DepthImageCollisionChecker, private pips::collision_testing::EgocylindricalImageCollisionChecker
{

public:
  DepthEgoCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME) :
    pips::collision_testing::TransformingCollisionChecker(nh,pnh,name),
    name_(name)
  {
      
  }
  
  void setCC(std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> cc1, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> cc2)
  {
    depth_cc_ = cc1;
    ego_cc_ = cc2;
  }
  
  void setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
  {
    
  }

  CCResult testCollisionImpl(PoseType pose, CCOptions options=CCOptions())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    
    //This is where the parallel cc testing goes
    CCResult res1,res2;
    
    #pragma omp parallel num_threads(2) //if(pose.position.x < .25)
    {
      #pragma omp single nowait
      {
        #pragma omp task shared(res1) //if(pose.position.x < .25)
        {
          res1 = depth_cc_->testCollisionImpl(pose, options);
        }
        #pragma omp task shared(res2) //if(pose.position.x < .25)
        {
          res2 = ego_cc_->testCollisionImpl(pose, options);
        }
      }
      #pragma omp taskwait
      
    }
    
    //auto t2 = ros::WallTime::now();
    
    //int64_t duration = (t2-t1).toNSec();
    //setup_durations_.addDuration(t1,t2);
    
    //ROS_DEBUG_STREAM_NAMED(name_ + ".setup_timing", "[" + name_ + "]: Setup Duration = " << setup_durations_.getLastDuration() << ", size=" << -1 << ", average setup duration = " << setup_durations_.averageDuration() );
    
    if(res1)
    {
      return res1;
    }
    else
    {
      return res2;
    }
  }

private:
  std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> depth_cc_, ego_cc_;
  //std::shared_ptr<pips::collision_testing::DepthImageCollisionChecker> depth_cc_;
  //std::shared_ptr<pips::collision_testing::EgocylindricalImageCollisionChecker> ego_cc_;
  static constexpr const char* DEFAULT_NAME="depth_ego_image_cc";
  std::string name_;
  //pips::utils::DurationAccumulator setup_durations_;
  
};





class DepthEgoCCWrapper : public pips_trajectory_testing::PipsCCWrapper
{

public:

  DepthEgoCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const int tamper_prevention=MAGIC_NUMBER,  std::shared_ptr<tf2_ros::Buffer> tf_buffer=std::make_shared<tf2_ros::Buffer>()) :
    PipsCCWrapper(nh,pnh,name,tf_buffer)
  {
      // If a tfbuffer was not provided by the user, then we need to set up a listener
      if(tamper_prevention == MAGIC_NUMBER)
      {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      }
      //Should probably add something here to throw an error, possibly compile time, if possible

  }


  DepthEgoCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& name=DEFAULT_NAME) :
    PipsCCWrapper(nh,pnh,name, tf_buffer)
  {
  }
  
  void setCC(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc1, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc2)
  {
    depth_cc_ = cc1;
    ec_cc_ = cc2;
    
    cc_=std::make_shared<DepthEgoCollisionChecker>(nh_, pnh_);
    cc_->setCC(depth_cc_->getCC(), ec_cc_->getCC());
  }

  void update()
  {
    ros::WallTime t1 = ros::WallTime::now();
    
    #pragma omp parallel num_threads(2)
    {
      #pragma omp sections
      {
        #pragma omp section
        {
          depth_cc_->update();
        }
        #pragma omp section
        {
          ec_cc_->update();
        }
      }      
    }
    
    auto t2 = ros::WallTime::now();
    setup_durations_.addDuration(t1,t2);
    
    ROS_DEBUG_STREAM_NAMED(name_ + ".setup_timing", "[" + name_ + "]: Setup Duration = " << setup_durations_.getLastDuration() << ", size=" << -1 << ", average setup duration = " << setup_durations_.averageDuration() );
  }

  bool isReady()
  {
    return depth_cc_->isReady() && ec_cc_->isReady();
  }

  bool isReady( const std_msgs::Header& header )
  {
    bool a = depth_cc_->isReady(header);
    bool b = ec_cc_->isReady(header);
    
    ROS_DEBUG_STREAM("Depth ready = " << a << ", ec ready = " << b);
    
    return a && b;
  }
  
  bool init()
  {
    bool a = depth_cc_->init();
    bool b = ec_cc_->init();
    
    return a && b;
  }

  virtual std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
  {
    return cc_;
  }
  
  virtual std_msgs::Header getCurrentHeader()
  {
    //Note: I'm not sure if this is the best option. Then again, this may never get called anyway
    return depth_cc_->getCurrentHeader();
  }
  
private:

  static constexpr const int MAGIC_NUMBER = -17;  //The sole purpose of this is to ensure that the constructor with the 'optional' tfbuffer argument is not accidentally passed a tfbuffer object
  std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> depth_cc_, ec_cc_;

  std::shared_ptr<pips_egocylindrical::DepthEgoCollisionChecker> cc_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  static constexpr const char* DEFAULT_NAME="depth_ego_image_cc_wrapper";
  pips::utils::DurationAccumulator setup_durations_;
  
  
};

}

#endif
