#include <pips_egocylindrical/egocylindrical_image_collision_checker.h>
#include <pips/collision_testing/image_geometry_models/image_geometry_converter.h>


namespace pips
{
  namespace collision_testing
  {
    

    EgocylindricalImageCollisionChecker::EgocylindricalImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm):
      PipsCollisionChecker(nh,pnh,tfm,name)
    {}

    EgocylindricalImageCollisionChecker::EgocylindricalImageCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name):
      PipsCollisionChecker(nh,pnh,tfm,name)
    {}

    std::shared_ptr<pips::utils::AbstractCameraModel> EgocylindricalImageCollisionChecker::getCameraModel()
    {
      cam_model_ = std::make_shared<pips::utils::EgocylindricalCameraModel>();
      return cam_model_;
    }
    
    void EgocylindricalImageCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg)
    {
      cam_model_->setInfo(info_msg);
      cam_model_->update();
      
      PipsCollisionChecker::setImage(image_msg);
    }

    void EgocylindricalImageCollisionChecker::initImpl()
    {
      PipsCollisionChecker::initImpl();

      search_radius_ = 1;
      pnh_.getParam("search_radius", search_radius_);
      pnh_.setParam("search_radius", search_radius_);
    }

    float EgocylindricalImageCollisionChecker::getFreeSpaceRadius(geometry_msgs::Pose pose)
    {
      ROS_DEBUG_STREAM_NAMED(name_+".freespace_radius","Pose: " << toString(pose));

      auto models = robot_model_->getModel<pips::collision_testing::image_geometry_models::ImageGeometryConverter>(pose);

      const auto& converter = cam_model_->getModel();


      float* cyl_image = (float*)this->image_ref_.data;


      auto get_num_row_inflation_indices = [converter](float range, float inflation_radius)
      {
        return std::atan2(range, inflation_radius) * converter.getHScale();
        //return std::asin(float(inflation_radius)/range) * converter.getHScale();
      };

      auto get_num_col_inflation_indices = [converter](float range, float inflation_radius)
      {
        return inflation_radius * converter.getVScale() / range;
      };

      auto get_row_search_size = [get_num_row_inflation_indices, this](float range)
      {
        return get_num_row_inflation_indices(range, search_radius_);
      };

      auto get_col_search_size = [get_num_col_inflation_indices, this](float range)
      {
        return get_num_col_inflation_indices(range, search_radius_);
      };

      auto get_min_depth = [this, converter, cyl_image, get_row_search_size, get_col_search_size](float x, float y, float z)
      {      
        cv::Point3f world_pt(x, y, z);
        cv::Point pix = converter.worldToCylindricalImage(world_pt);

        float range = egocylindrical::utils::worldToRange(world_pt);

        int image_height = converter.getHeight();
        int image_width = converter.getWidth();

        int x_search_window = get_row_search_size(range);
        int y_search_window = get_col_search_size(range);

        int startx = pix.x-x_search_window;
        int endx = pix.x+x_search_window;

        int starty = std::max(pix.y-y_search_window, 0);
        int endy = std::min(pix.y+y_search_window, image_height-1);

        int closestx, closestz;
        float min_depth = std::numeric_limits<float>::max();

        for(int yind = starty; yind <= endy; yind++)
        {
          for(int xind = startx; xind <= endx; xind++)
          {
            int mod_x = (xind + image_width) % image_width;
            int ind = converter.pixToIdx(mod_x, yind);
            float depth = cyl_image[ind];
            if(depth > 0 && depth < min_depth)
            {
              min_depth = depth;
            }
            //min_depth = std::min(depth, min_depth);
          }
        }

        return min_depth;
      };

      float min_depth = std::numeric_limits<float>::max();

      for(const auto& model : models)
      {      
        auto* cyl = (pips::collision_testing::image_geometry_models::Cylinder*)model.get();
        float height = cyl->height_;
        float radius = cyl->radius_;

        auto p = model->pose_.position;

        float model_min_depth = get_min_depth(p.x, p.y, p.z);

        min_depth = std::min(min_depth, model_min_depth);

      }

      //done:

      return min_depth;
    }

    
  
  }
  
}
