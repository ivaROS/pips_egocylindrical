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
        
        cv::Point2d project3dToPixel(const cv::Point3d& point)
        {
          return model_.project3dToPixel(point);
        }
        
        cv::Point3d projectPixelTo3dRay(const cv::Point2d& point)
        {
          return model_.projectPixelTo3dRay(point);
        }
        
        std::vector<int> getColumnRange(int left, int right)
        {          
          std::vector<int> cols;
          
          if(left < right)
          {
            for(int i = left; left < right; ++i)
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
        
        float getPixelValue(const cv::Point3d& point)
        {
          return point.z;
        }
      
      
    };
  
  }


}


#endif  // PIPS_UTILS_EGOCYLINDRICAL_CAMERA_MODEL
