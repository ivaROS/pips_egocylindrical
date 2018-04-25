#ifndef DEPTH_IMAGE_CC_WRAPPER_H
#define DEPTH_IMAGE_CC_WRAPPER_H

#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <pips_egocylindrical/egocylindrical_image_collision_checker.h>
#include <sensor_msgs/Image.h>
//#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


namespace pips_egocylindrical
{

class EgocylindricalRangeImageCCWrapper : public pips_trajectory_testing::PipsCCWrapper
{

  
private:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> exact_image_sync_policy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> approx_image_sync_policy;
  typedef message_filters::Synchronizer<approx_image_sync_policy> time_synchronizer;
    
  typedef tf2_ros::MessageFilter<egocylindrical::EgoCylinderPoints> tf_filter;
  
  
  boost::shared_ptr<tf_filter> info_tf_filter_;
  boost::shared_ptr<time_synchronizer> image_synchronizer_;
    
  image_transport::SubscriberFilter ec_sub_;
  message_filters::Subscriber<egocylindrical::EgoCylinderPoints> ec_info_sub_;
  
  sensor_msgs::Image::ConstPtr current_image;
  egocylindrical::EgoCylinderPoints::ConstPtr current_camInfo;
  
  std::shared_ptr<pips::collision_testing::EgocylindricalImageCollisionChecker> cc_;
  
    
  void ecImageCb(const sensor_msgs::Image::ConstPtr& image_msg,
                                                  const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg);
  
public:
  EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer);
  
  
  bool init();
  
  void update();

  
  std_msgs::Header getCurrentHeader();
  
  std::shared_ptr<PipsCollisionChecker> getCC()
  {
    return cc_;
  }
  

};

}

#endif //DEPTH_IMAGE_CC_WRAPPER_H