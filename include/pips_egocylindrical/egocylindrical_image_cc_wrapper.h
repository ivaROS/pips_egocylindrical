#ifndef EGOCYLINDRICAL_IMAGE_CC_WRAPPER_H
#define EGOCYLINDRICAL_IMAGE_CC_WRAPPER_H

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
public:
  static constexpr const char* DEFAULT_NAME="egocylindrical_image_cc_wrapper";
  
private:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> exact_image_sync_policy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> approx_image_sync_policy;
  typedef message_filters::Synchronizer<approx_image_sync_policy> time_synchronizer;
    
  typedef tf2_ros::MessageFilter<egocylindrical::EgoCylinderPoints> tf_filter;
  typedef pips_trajectory_testing::PipsCCWrapper Super;
  
  boost::shared_ptr<tf_filter> info_tf_filter_;
  boost::shared_ptr<time_synchronizer> image_synchronizer_;
    
  image_transport::SubscriberFilter ec_sub_;
  message_filters::Subscriber<egocylindrical::EgoCylinderPoints> ec_info_sub_;
  
  sensor_msgs::Image::ConstPtr current_image;
  egocylindrical::EgoCylinderPoints::ConstPtr current_camInfo;
  
  std::shared_ptr<pips::collision_testing::EgocylindricalImageCollisionChecker> cc_;
  
  
public:
  EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
  
  EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);

  bool init();
  
  void update();

  bool isReadyImpl();
  
  std_msgs::Header getCurrentHeader();
  
  std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
  {
    return cc_;
  }
  
private:
  void ecImageCb(const sensor_msgs::Image::ConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg);
  
};

}

#endif //EGOCYLINDRICAL_IMAGE_CC_WRAPPER_H
