
#include <pips_egocylindrical/egocan_cc_wrapper.h>

namespace pips_egocylindrical
{

  EgoCanCCWrapper::EgoCanCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
  PipsCCWrapper(nh,pnh,name,tfm)
{
    cc_ = std::make_shared<pips_egocylindrical::EgoCanCollisionChecker>(nh, pnh_, tfm);
}


EgoCanCCWrapper::EgoCanCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
  PipsCCWrapper(nh,pnh,name,tfm)
{
    cc_ = std::make_shared<pips_egocylindrical::EgoCanCollisionChecker>(nh, pnh_, tfm);
}

bool EgoCanCCWrapper::init()
{    
  if(!inited_)
  {
    PipsCCWrapper::init();
  
    // Get topic names
    std::string egocan_image_topic="/egocylinder/can_image", egocan_info_topic= "/egocylinder/egocylinder_info";
    
    /*
    pnh_.getParam("egocan_image_topic", egocan_image_topic );
    pnh_.getParam("egocan_info_topic", egocan_info_topic );
    
        // The idea here is to set the parameter on the parameter server to the default value to make it easier to see what it is.
    pnh_.setParam("egocan_image_topic", egocan_image_topic);
    pnh_.setParam("egocan_info_topic", egocan_info_topic );
    */
        
    bool res = pips::utils::get_param(pnh_, "egocan_image_topic", egocan_image_topic, egocan_image_topic, 10);
    res &= pips::utils::get_param(pnh_, "egocan_info_topic", egocan_info_topic, egocan_info_topic, 10);
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    image_transport::ImageTransport it ( nh_ );
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );

    ec_sub_.subscribe(it, egocan_image_topic, 1);
    ec_info_sub_.subscribe(nh_, egocan_info_topic, 1);

    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(ec_info_sub_, *tfm_.getBuffer(), PipsCCWrapper::fixed_frame_id_, 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),ec_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&EgoCanCCWrapper::ecImageCb, this, _1, _2));
    
    free_space_checking_service_ = pnh_.advertiseService("free_space_checker", &EgoCanCCWrapper::freeSpaceCheckingSrv, this);
    
    
    inited_ = true;
  }
  return true;

}


bool EgoCanCCWrapper::freeSpaceCheckingSrv(FreeSpaceCheckerService::Request &req, FreeSpaceCheckerService::Response &res)
{
  if(!cc_)
  {
    ROS_ERROR_STREAM("Unable to test freespace without collision checker!");
    return false;
  }
  if(transformReady(getCurrentHeader(), req.pose.header))
  {
    res.distance = cc_->getFreeSpaceRadius(req.pose.pose);
    return true;
  }
  else
  {
    return false;
  }
}


void EgoCanCCWrapper::update()
{
    ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker image" );
    cc_->setImage ( current_image, current_camInfo );
}


bool EgoCanCCWrapper::isReadyImpl()
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
void EgoCanCCWrapper::ecImageCb (const sensor_msgs::Image::ConstPtr& image_msg,
                                                  const egocylindrical::EgoCylinderPoints::ConstPtr& info_msg)
{
    ROS_DEBUG_STREAM_NAMED(name_ + ".image_callback", "Received synchronized messages with image stamp " << image_msg->header.stamp << " and info stamp " << info_msg->header.stamp);
    if ( image_msg->header.stamp == ros::Time ( 0 ) || info_msg->header.stamp == ros::Time ( 0 ) ) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
        ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
        return;
    }


    current_image = image_msg;
    current_camInfo = info_msg;

    doCallback();
}

std_msgs::Header EgoCanCCWrapper::getCurrentHeader()
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

