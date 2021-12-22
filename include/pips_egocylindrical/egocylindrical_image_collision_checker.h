#ifndef EGOCYLINDRICAL_IMAGE_COLLISION_CHECKER_H
#define EGOCYLINDRICAL_IMAGE_COLLISION_CHECKER_H

#include <pips/collision_testing/pips_collision_checker.h>
#include <egocylindrical/ecwrapper.h>
#include <pips_egocylindrical/egocylindrical_camera_model.h>
#include <pips_egocylindrical/free_space_checker.h>

namespace pips
{
  namespace collision_testing
  {
    
    class EgocylindricalImageCollisionChecker : public PipsCollisionChecker, public pips_egocylindrical::FreeSpaceChecker
    {
    protected:
      std::shared_ptr<pips::utils::EgocylindricalCameraModel> cam_model_;
      
      
    public:
      
      EgocylindricalImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
      EgocylindricalImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
      
      virtual void initImpl();
      
      void setImage(const sensor_msgs::ImageConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg);
      float getFreeSpaceRadius(geometry_msgs::Pose pose);
      
      static constexpr const char* DEFAULT_NAME="egocylindrical_image_collision_checker";

      
    private:
      std::shared_ptr<pips::utils::AbstractCameraModel> getCameraModel();
      float search_radius_;
      
    };
  }
}



#endif //EGOCYLINDRICAL_IMAGE_COLLISION_CHECKER_H

