#include "lk_vio/ros_utilities.hpp"

#include <thread>
namespace lk_vio
{
  RosUtilities::RosUtilities( const common::Configuration &config )
  {
    nh_ = ros::NodeHandle( "~" );

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/lk_vio/pose", 1 );

    static image_transport::ImageTransport it_( nh_ );
    image_pub_ = it_.advertise( "/lk_vio/image", 1 );

    current_points_pub_   = nh_.advertise<sensor_msgs::PointCloud>( "/lk_vio/current_points", 1 );
    global_points_pub_    = nh_.advertise<sensor_msgs::PointCloud>( "/lk_vio/global_points", 1 );
    key_frame_points_pub_ = nh_.advertise<sensor_msgs::PointCloud>( "/lk_vio/key_frame_points", 1 );

    key_frame_path_pub_ = nh_.advertise<nav_msgs::Path>( "/lk_vio/key_frame_path", 1 );
    current_path_pub_   = nh_.advertise<nav_msgs::Path>( "/lk_vio/path", 1 );
    global_path_        = nav_msgs::Path();
    key_frame_path_     = nav_msgs::Path();

    map_ = nullptr;
  }

  void RosUtilities::publishPointCloud( const std::vector<PointXYZ> &points )
  {
    if ( points.size() == 0 )
    {
      return;
    }

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp    = ros::Time::now();

    for ( const auto &point : points )
    {
      geometry_msgs::Point32 p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      point_cloud.points.push_back( p );
    }

    if ( global_points_pub_.getNumSubscribers() > 0 )
    {
      global_points_pub_.publish( point_cloud );
    }
  }

  void RosUtilities::addCurrentFrame( const std::shared_ptr<lk_vio::Frame> &frame, const std::vector<uchar> &status )
  {
    // extract pose
    auto pose_stamped = extractPoseFromFrame( frame );
    publishPose( pose_stamped );

    // Publish Path
    updateGlobalPath( pose_stamped );
    if ( current_path_pub_.getNumSubscribers() > 0 )
    {
      // std::cout << "publishing path" << std::endl;
      current_path_pub_.publish( global_path_ );
    }

    // TODO: publish image with ORB and LK-Flow
    int     numChannels = frame->left_image_.channels();
    cv::Mat show_left_img( frame->left_image_.size(), frame->left_image_.type() );
    cv::Mat show_right_img( frame->right_image_.size(), frame->right_image_.type() );

    // Verify the number of channels
    if ( numChannels != 3 )
    {
      cv::cvtColor( frame->left_image_, show_left_img, cv::COLOR_GRAY2BGR );
      cv::cvtColor( frame->right_image_, show_right_img, cv::COLOR_GRAY2BGR );
    }
    else
    {
      show_left_img = frame->left_image_.clone();
    }

    if ( frame->features_right_.size() > 0 )
    {
      // std::cout << "visualize lk\n";
      for ( size_t i = 0; i < frame->features_left_.size(); i++ )
      {
        if ( status[ i ] )
        {
          cv::Point2i pt1 = frame->features_left_[ i ]->kp_position_.pt;
          cv::Point2i pt2 = frame->features_right_[ i ]->kp_position_.pt;
          cv::circle( show_left_img, pt1, 3, cv::Scalar( 0, 0, 255 ), 5 );
          cv::line( show_left_img, pt1, pt2, cv::Scalar( 0, 255, 0 ), 3 );
        }
      }
    }
    else
    {
      for ( const auto &feat : frame->features_left_ )
      {
        cv::Point2i pt1 = feat->kp_position_.pt;
        cv::circle( show_left_img, pt1, 2, cv::Scalar( 0, 0, 255 ), 2 );
      }
    }

    cv::Mat combined_img;
    cv::vconcat( show_left_img, show_right_img, combined_img );
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage( std_msgs::Header(), "bgr8", combined_img ).toImageMsg();
    image_pub_.publish( msg );

    // cv::Size new_size( 1280, 480 );
    // cv::resize( combined_img, combined_img, new_size, 0, 0, cv::INTER_LINEAR );

    // cv::namedWindow( "Current Frame", cv::WINDOW_AUTOSIZE );
    // cv::imshow( "Current Frame", combined_img );
    // cv::waitKey( 1 );
    // Publish Map
    this->publishMap();
  }

  geometry_msgs::PoseStamped RosUtilities::extractPoseFromFrame( const std::shared_ptr<lk_vio::Frame> &frame )
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp    = ros::Time().fromSec( frame->timestamp_ );

    auto pose = frame->getPose().inverse();

    pose_stamped.pose.position.x    = pose.translation().x();
    pose_stamped.pose.position.y    = pose.translation().y();
    pose_stamped.pose.position.z    = pose.translation().z();
    pose_stamped.pose.orientation.x = pose.unit_quaternion().x();
    pose_stamped.pose.orientation.y = pose.unit_quaternion().y();
    pose_stamped.pose.orientation.z = pose.unit_quaternion().z();
    pose_stamped.pose.orientation.w = pose.unit_quaternion().w();

    return pose_stamped;
  }

  void RosUtilities::publishPose( const geometry_msgs::PoseStamped &pose )
  {
    pose_pub_.publish( pose );
  }

  void RosUtilities::updateGlobalPath( const geometry_msgs::PoseStamped &pose_stamped )
  {
    // Publish path
    global_path_.header.frame_id = "world";
    global_path_.header.stamp    = pose_stamped.header.stamp;
    global_path_.poses.push_back( pose_stamped );
  }

  void RosUtilities::setMap( const std::shared_ptr<lk_vio::Map> map )
  {
    assert( map != nullptr );
    map_ = map;
  }

  void RosUtilities::publishMap()
  {
    if ( map_ == nullptr )
    {
      std::cout << "Map is not set." << std::endl;
      return;
    }

    if ( map_->GetActiveKeyFrames().size() == 0 )
    {
      return;
    }

    if ( map_->GetAllKeyFrames().size() == 0 )
    {
      return;
    }

    if ( map_->GetAllMapPoints().size() == 0 )
    {
      return;
    }


    std::lock_guard<std::mutex> lock( map_->mmutex_map_update_ );

    // std::cout << "map key frame size: " << map_->GetAllKeyFrames().size() << std::endl;
    // std::cout << "map map point size: " << map_->GetAllMapPoints().size() << std::endl;
    // publish key frame path

    if ( key_frame_path_pub_.getNumSubscribers() > 0 )
    {
      key_frame_path_ = nav_msgs::Path();

      key_frame_path_.header.frame_id = "world";
      key_frame_path_.header.stamp    = ros::Time::now();  //


      for ( auto &kf : map_->GetAllKeyFrames() )
      {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp    = ros::Time().fromSec( kf.second->timestamp_ );

        auto pose = kf.second->getPose().inverse();

        pose_stamped.pose.position.x    = pose.translation().x();
        pose_stamped.pose.position.y    = pose.translation().y();
        pose_stamped.pose.position.z    = pose.translation().z();
        pose_stamped.pose.orientation.x = pose.unit_quaternion().x();
        pose_stamped.pose.orientation.y = pose.unit_quaternion().y();
        pose_stamped.pose.orientation.z = pose.unit_quaternion().z();
        pose_stamped.pose.orientation.w = pose.unit_quaternion().w();

        key_frame_path_.header.frame_id = "world";
        key_frame_path_.header.stamp    = ros::Time().fromSec( kf.second->timestamp_ );
        key_frame_path_.poses.push_back( pose_stamped );
      }

      key_frame_path_pub_.publish( key_frame_path_ );
    }


    // publish key frame points
    if ( key_frame_points_pub_.getNumSubscribers() > 0 )
    {
      key_frame_points_                 = sensor_msgs::PointCloud();
      key_frame_points_.header.frame_id = "world";

      for ( auto &map_point : map_->GetAllMapPoints() )
      {
        auto                   point = map_point.second->getPosition();
        geometry_msgs::Point32 p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        key_frame_points_.points.push_back( p );
      }

      key_frame_points_pub_.publish( key_frame_points_ );
    }
  }
}  // namespace lk_vio