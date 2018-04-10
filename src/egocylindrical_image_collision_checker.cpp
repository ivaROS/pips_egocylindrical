#include <pips_egocylindrical/egocylindrical_image_collision_checker.h>



namespace pips
{
  namespace collision_testing
  {
    

    EgocylindricalImageCollisionChecker::EgocylindricalImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh) : 
      PipsCollisionChecker(nh,pnh)
    {
      
    }

    std::shared_ptr<pips::utils::AbstractCameraModel> EgocylindricalImageCollisionChecker::getCameraModel()
    {
      cam_model_ = std::make_shared<pips::utils::EgocylindricalCameraModel>();
      return cam_model_;
    }
    
    void EgocylindricalImageCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg)
    {
      cam_model_->setInfo(info_msg);
      
      PipsCollisionChecker::setImage(image_msg);
    }
    
  
  }
  
}