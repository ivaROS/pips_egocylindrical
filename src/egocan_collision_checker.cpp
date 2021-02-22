#include <pips_egocylindrical/egocan_collision_checker.h>
#include <pips/collision_testing/image_geometry_models/image_geometry_converter.h>
#include <pips/utils/image_comparison_implementations.h>

#include <pips/GenerateDepthImage.h>
#include <pips/TestCollision.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/assert.h>
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
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


#define SCALE_METERS 1
#define SCALE_MM 1000

  
  

namespace pips_egocylindrical
{

  EgoCanCollisionChecker::EgoCanCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) : 
    pips::collision_testing::GeneralCollisionChecker(nh,pnh,name,tfm)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".constructor", "Constructing collision checker");
    //show_im_=true;
    //transpose_=false;
  }
  
  EgoCanCollisionChecker::EgoCanCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) : 
    pips::collision_testing::GeneralCollisionChecker(nh,pnh,name,tfm)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".constructor", "Constructing collision checker");
    //show_im_=true;
    //transpose_=false;
  }

  void EgoCanCollisionChecker::initImpl()
  {
    pips::collision_testing::GeneralCollisionChecker::initImpl();
    //cam_model_ = getCameraModel();
    
    search_radius_ = 1;
    pnh_.getParam("search_radius", search_radius_);
    pnh_.setParam("search_radius", search_radius_);
    
    //TODO: This should probably accept a CameraInfo message as an optional parameter, allowing it to be used without a camera
    //depth_generation_service_ = pnh_.advertiseService("generate_depth_image", &PipsCollisionChecker::getDepthImageSrv, this);
  }
  

  void EgoCanCollisionChecker::setImage(const sensor_msgs::ImageConstPtr& image_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& info)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".setup", "Setting new image" << std::endl);
    
    auto t1 = ros::WallTime::now();
    
    try {
      
      if(image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        input_bridge_ref_ = cv_bridge::toCvShare(image_msg);
        image_ref_ = input_bridge_ref_->image;
        scale_ = SCALE_METERS;
      }
      else if(image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        ROS_ASSERT_MSG(false, "16UC1 images cannot currently be used with EgoCanCollisionChecker!");
        ROS_ERROR_NAMED("setImage", "This hasn't been written to fully work with 16UC1 images yet!");
        //Make a copy of depth image and set all 0's (unknowns) in image to some large value.
        cv_bridge::CvImagePtr cv_im = cv_bridge::toCvCopy(image_msg);
        scale_ = SCALE_MM;
        
        image_ref_ = cv_im->image;
        image_ref_.setTo(std::numeric_limits<uint16_t>::max(), image_ref_==0);
        
        input_bridge_ref_ = cv_im;
      }
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR_NAMED(name_+".setup", "Failed to convert image");
      return;
    }
    
    converter_.fromCameraInfo(info);
    
    auto t2 = ros::WallTime::now();
    
    setup_durations_.addDuration(t1,t2);
    
    ROS_DEBUG_STREAM_NAMED(name_ + ".setup_timing", "[" + name_ + "]: Setup Duration = " << setup_durations_.getLastDuration() << ", size=" << input_bridge_ref_->image.cols*input_bridge_ref_->image.rows << ", average setup duration = " << setup_durations_.averageDuration() );
    
    return;
  }

      
  CCResult EgoCanCollisionChecker::testCollisionImpl(geometry_msgs::Pose pose, CCOptions options)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".collision_test","Pose: " << toString(pose));
    
    auto models = robot_model_.getModel<pips::collision_testing::image_geometry_models::ImageGeometryConverter>(pose);
    
    int can_width = converter_.getCanWidth();
    
    float* can_image = (float*)this->image_ref_.data;
    
    CCResult result;
    for(const auto& model : models)
    {      
      auto* cyl = (pips::collision_testing::image_geometry_models::Cylinder*)model.get();
      float height = cyl->height_;
      float radius = cyl->radius_;
      
      auto p = model->pose_.position;
      
      //Check bottom face:
      {
        double bottom_face_model_depth = p.y + height/2;
        int ind = converter_.worldToCanIdx(p.x, bottom_face_model_depth, p.z);
        if(ind >= converter_.getCols() && ind < converter_.getNumPts())
        {
          int bottom_img_ind = ind - converter_.getCols();
          
          float bottom_face_world_depth = can_image[bottom_img_ind];
          
          if(bottom_face_model_depth >= bottom_face_world_depth)
          {
            cv::Point pt(bottom_img_ind % can_width, bottom_img_ind / can_width);
            auto ray = converter_.projectCanPixelTo3dRay(pt);
            auto colliding_pt = ray * bottom_face_world_depth;
            result.addPoint(pips::collision_testing::toCollisionPoint(colliding_pt));
            
            //goto done;
          }
        }
      }
      
      //Check top face:
      {
        double top_face_model_depth = -p.y + height/2;
        int ind = converter_.worldToCanIdx(p.x, top_face_model_depth, p.z);
        if(ind >= converter_.getCols() && ind < converter_.getNumPts())
        {
          
          int top_img_ind = ind - converter_.getCols();
          
          float top_face_world_depth = can_image[top_img_ind];
          
          if(top_face_model_depth >= top_face_world_depth)
          {
            cv::Point pt(top_img_ind % can_width, top_img_ind / can_width);
            auto ray = converter_.projectCanPixelTo3dRay(pt);
            auto colliding_pt = ray * top_face_world_depth;
            result.addPoint(pips::collision_testing::toCollisionPoint(colliding_pt));
            
            //goto done;
          }
        }
      }
      
      
    }
    
    //done:
    
    return result;
  }
  
  
  float EgoCanCollisionChecker::getFreeSpaceRadius(geometry_msgs::Pose pose)
  {
    ROS_DEBUG_STREAM_NAMED(name_+".freespace_radius","Pose: " << toString(pose));
    
    auto models = robot_model_.getModel<pips::collision_testing::image_geometry_models::ImageGeometryConverter>(pose);
    
    int can_width = converter_.getCanWidth();
    
    float* can_image = (float*)this->image_ref_.data;
    
    
    auto get_num_inflation_indices = [this](float depth, float inflation_radius)
    {
      return inflation_radius * converter_.getCanScale() / std::abs(depth);
    };
    
    auto get_search_size = [get_num_inflation_indices, this](float depth)
    {
      return get_num_inflation_indices(depth, search_radius_);
    };
    
    auto get_min_depth = [this, can_image, get_search_size, can_width](float x, float y, float z)
    {      
      int search_window = get_search_size(y);
      
      int xpix = converter_.worldToCanXIdx(x, y, z);
      int zpix = converter_.worldToCanZIdx(x, y, z);
      
      int startx = std::max(xpix-search_window, 0);
      int endx = std::min(xpix+search_window, can_width-1);
      
      int startz = std::max(zpix-search_window, 0);
      int endz = std::min(zpix+search_window, can_width-1);
      
      int closestx, closestz;
      float min_depth = std::numeric_limits<float>::max();
      
      for(int xind = startx; xind <= endx; xind++)
      {
        for(int zind = startz; zind <= endz; zind++)
        {
          int can_ind = converter_.pixToCanIdx(xind, zind, y);
          float depth = can_image[can_ind];
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
      
      //Check bottom and top faces:
      {
        float bottom_face_model_depth = p.y + height/2;
        
        float min_bottom_depth = get_min_depth(p.x, bottom_face_model_depth, p.z);
        
        min_depth = std::min(min_depth, min_bottom_depth);
        
        float top_face_model_depth = p.y - height/2;
        float min_top_depth = get_min_depth(p.x, top_face_model_depth, p.z);
        
        min_depth = std::min(min_depth, min_top_depth);
        
      }

    }
    
    //done:
    
    return min_depth;
  }
  
  
  
  std_msgs::Header EgoCanCollisionChecker::getCurrentHeader()
  {
    if(input_bridge_ref_)
    {
      return input_bridge_ref_->header;
    }
    else
    {
      return std_msgs::Header();
    }
  }
  
}

