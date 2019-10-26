#ifndef PIPS_UTILS_EGOCYLINDRICAL_CAMERA_MODEL
#define PIPS_UTILS_EGOCYLINDRICAL_CAMERA_MODEL


#include <pips/utils/abstract_camera_model.h>
#include <egocylindrical/ecwrapper.h>
//#include <opencv2/core/core.hpp>


namespace pips
{
  namespace utils
  {
    
      
    class EgocylindricalCameraModel : public AbstractCameraModel
    {
      private:
        egocylindrical::utils::ECConverter model_;
        egocylindrical::EgoCylinderPoints::ConstPtr info_msg_;
        
      public:
        
        void setInfo(const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg)
        {
          info_msg_ = info_msg;
        }
        
        void update()
        {
          model_.fromCameraInfo(info_msg_);
        }
        
        cv::Point2d project3dToPixel(const cv::Point3d& point) const
        {
          
          /*
          // Verified that functions are inverses of each other
          cv::Point2d projected = model_.project3dToPixel(point);
          
          double dist = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
          cv::Point3d ray = point / dist;
          
          cv::Point3d pixel_ray = projectPixelTo3dRay(projected);
          
          ROS_INFO_STREAM("World point: " << point << ", unit vector: " << ray << ", pixel: " << projected << ", pixel_ray: " << pixel_ray);
          */
          
          return model_.project3dToPixel(point);
        }
        
        cv::Point3d projectPixelTo3dRay(const cv::Point2d& point) const
        {
          return model_.projectPixelTo3dRay(point);
        }
        
        std::vector<int> getColumnRange(int left, int right) const
        {          
          std::vector<int> cols;
          
          if(left < right)
          {
            for(int i = left; i < right; ++i)
            {
              cols.push_back(i);
            }
          }
          else
          {
            for(int i = left; i < model_.getWidth(); ++i)
            {
              cols.push_back(i);
            }
            for(int i = 0; i < right; ++i)
            {
              cols.push_back(i);
            }
          }
          
          return cols;
        }
        
        float getPixelValue(const cv::Point3d& point) const
        {
          return egocylindrical::utils::worldToRange(point);
        }
      
      
    };
  
  }


}


#endif  // PIPS_UTILS_EGOCYLINDRICAL_CAMERA_MODEL
