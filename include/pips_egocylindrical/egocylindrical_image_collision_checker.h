#ifndef EGOCYLINDRICAL_IMAGE_COLLISION_CHECKER_H
#define EGOCYLINDRICAL_IMAGE_COLLISION_CHECKER_H

#include <pips/collision_testing/pips_collision_checker.h>
#include <egocylindrical/ecwrapper.h>
#include <pips_egocylindrical/egocylindrical_camera_model.h>

namespace pips
{
  namespace collision_testing
  {
    
    class EgocylindricalImageCollisionChecker : public PipsCollisionChecker
    {
    private:
      std::shared_ptr<pips::utils::EgocylindricalCameraModel> cam_model_;
      
      
    public:
      
      EgocylindricalImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh);
      
      
      void setImage(const sensor_msgs::ImageConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg);
      
      
    private:
      std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel();
      
      
    };
  }
}



#endif //EGOCYLINDRICAL_IMAGE_COLLISION_CHECKER_H

