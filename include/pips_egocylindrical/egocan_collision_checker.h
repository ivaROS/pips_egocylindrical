#ifndef PIPS_EGOCYLINDRICAL_EGOCAN_COLLISION_CHECKER_H
#define PIPS_EGOCYLINDRICAL_EGOCAN_COLLISION_CHECKER_H

#include <egocylindrical/ecwrapper.h>
#include <pips/collision_testing/pips_collision_checker.h>
#include <pips_egocylindrical/free_space_checker.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::min

#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sensor_msgs/image_encodings.h>

#include <climits>
#include <limits>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>

//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>



namespace pips_egocylindrical
{

class EgoCanCollisionChecker : public pips::collision_testing::GeneralCollisionChecker, public pips_egocylindrical::FreeSpaceChecker
{
public:
  EgoCanCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& manager=tf2_utils::TransformManager(false));
  
  EgoCanCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);

  void initImpl();

  void setImage(const sensor_msgs::ImageConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info);

  CCResult testCollisionImpl(geometry_msgs::Pose pose, CCOptions options);
  
  float getFreeSpaceRadius(geometry_msgs::Pose pose);
  
  std_msgs::Header getCurrentHeader();

protected:
  unsigned int scale_;
  
  float search_radius_;
  
  static constexpr const char* DEFAULT_NAME="egocan_collision_checker";
  
  cv::Mat image_,image_ref_;
  
  cv_bridge::CvImageConstPtr input_bridge_ref_;
  
  cv_bridge::CvImagePtr input_bridge_, output_bridge_;
  
  egocylindrical::utils::ECConverter converter_;
};
  

} //end namespace pips_egocylindrical
  


#endif //PIPS_EGOCYLINDRICAL_EGOCAN_COLLISION_CHECKER_H
