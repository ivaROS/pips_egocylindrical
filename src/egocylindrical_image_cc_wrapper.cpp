
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>

namespace pips_egocylindrical
{

EgocylindricalRangeImageCCWrapper::EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
  PipsCCWrapper(nh,pnh,name,tfm)
{
    cc_ = std::make_shared<pips::collision_testing::EgocylindricalImageCollisionChecker>(nh, pnh_);
}


EgocylindricalRangeImageCCWrapper::EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
  PipsCCWrapper(nh,pnh,name,tfm)
{
    cc_ = std::make_shared<pips::collision_testing::EgocylindricalImageCollisionChecker>(nh, pnh_);
}

bool EgocylindricalRangeImageCCWrapper::init()
{    
  if(!inited_)
  {
    PipsCCWrapper::init();
  
    // Get topic names
    std::string depth_image_topic="egocylinder/image", depth_info_topic= "/egocylinder/data";
    
    pnh_.getParam("egocylindrical_image_topic", depth_image_topic );
    pnh_.getParam("egocylindrical_info_topic", depth_info_topic );
    
        // The idea here is to set the parameter on the parameter server to the default value to make it easier to see what it is.
    pnh_.setParam("egocylindrical_image_topic", depth_image_topic);
    pnh_.setParam("egocylindrical_info_topic", depth_info_topic );
    
    // TODO: use parameters for base_frame_id and odom_frame_id
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    image_transport::ImageTransport it ( nh_ );
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );

    ec_sub_.subscribe(it, depth_image_topic, 1);
    ec_info_sub_.subscribe(nh_, depth_info_topic, 1);

    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(ec_info_sub_, *tfm_.getBuffer(), PipsCCWrapper::fixed_frame_id_, 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),ec_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&EgocylindricalRangeImageCCWrapper::ecImageCb, this, _1, _2));
    
    inited_ = true;
  }
  return true;

}


void EgocylindricalRangeImageCCWrapper::update()
{
    ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker image" );
    cc_->setImage ( current_image, current_camInfo );
}


bool EgocylindricalRangeImageCCWrapper::isReadyImpl()
{
  if(current_camInfo && current_image)
  {
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_COND_NAMED(!current_camInfo, name_, "No current CameraInfo message!");
    ROS_ERROR_STREAM_COND_NAMED(!current_image, name_, "No current Image message!");
  }
  return false;
}
    
// TODO: Add a mutex to coordinate 'update' and 'depthImageCb'
// Could separate the image and camera_info callbacks to allow non synchronized messages
void EgocylindricalRangeImageCCWrapper::ecImageCb (const sensor_msgs::Image::ConstPtr& image_msg,
                                                  const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg)
{
    if ( image_msg->header.stamp == ros::Time ( 0 ) || info_msg->header.stamp == ros::Time ( 0 ) ) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
        ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
        return;
    }


    current_image = image_msg;
    current_camInfo = info_msg;

    doCallback();
}

std_msgs::Header EgocylindricalRangeImageCCWrapper::getCurrentHeader()
{
    if(current_camInfo)
    {
        return current_camInfo->header;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(name_, "Trying to get current header, but none exists!");
      return std_msgs::Header();
    }
}


}

