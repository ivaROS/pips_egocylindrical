
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>

namespace pips_egocylindrical
{

EgocylindricalRangeImageCCWrapper::EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    PipsCCWrapper(nh,pnh,"depth_image_cc_wrapper")
{
    cc_ = std::make_shared<pips::collision_testing::EgocylindricalImageCollisionChecker>(nh, pnh);
}


EgocylindricalRangeImageCCWrapper::EgocylindricalRangeImageCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer) :
    PipsCCWrapper(nh,pnh,"depth_image_cc_wrapper", tf_buffer)
{
    cc_ = std::make_shared<pips::collision_testing::EgocylindricalImageCollisionChecker>(nh, pnh);
}

bool EgocylindricalRangeImageCCWrapper::init()
{    
    // Get topic names
    std::string depth_image_topic="/range_image", depth_info_topic= "/egocylindrical_points";
    
    pnh_.getParam("egocylindrical_image_topic", depth_image_topic );
    pnh_.getParam("egocylindrical_info_topic", depth_info_topic );
    
    // TODO: use parameters for base_frame_id and odom_frame_id
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    image_transport::ImageTransport it ( nh_ );
    
    
    //pub_trajectoryProjection = it.advertise ( "trajectoryProjection", 1 );

    ec_sub_.subscribe(it, depth_image_topic, 1);
    ec_info_sub_.subscribe(nh_, depth_info_topic, 1);

    // Ensure that CameraInfo is transformable
    info_tf_filter_ = boost::make_shared<tf_filter>(ec_info_sub_, *tf_buffer_, "odom", 2,nh_);
    
    // Synchronize Image and CameraInfo callbacks
    image_synchronizer_ = boost::make_shared<time_synchronizer>(time_synchronizer(10),ec_sub_, *info_tf_filter_);
    image_synchronizer_->registerCallback(boost::bind(&EgocylindricalRangeImageCCWrapper::ecImageCb, this, _1, _2));

    return true;

}


void EgocylindricalRangeImageCCWrapper::update()
{
    if ( isReady ( current_image->header ) ) 
    {
        cc_->setImage ( current_image, current_camInfo );
        /*
        if ( _visualize ) {
            boost::mutex::scoped_lock l ( copy_mutex );
            imageVisualize = current_raw_image.clone();
            trajectoryProjectionHeader = current_image->header;
            visualizeReady = true;
        }
        */
    }
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

    // TODO: Not sure if this condition is necessary
    //if ( isReady ( image_msg->header ) ) 
    {

        //Update tester with new data
        ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker image" );
        current_image = image_msg;
        current_camInfo = info_msg;
        /*
        if ( _visualize ) {
            try {
                boost::shared_ptr<void const> tracked_object_disp;
                boost::mutex::scoped_lock l ( copy_mutex );
                current_raw_image = cv_bridge::toCvCopy ( *raw_image_msg, sensor_msgs::image_encodings::BGR8 )->image;
            } catch ( ... ) {
                ROS_ERROR ( "Exception within cost map callback function\n" );
            }
        }
        */


    }
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
        return std_msgs::Header();
    }
}


}

