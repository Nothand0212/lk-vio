#include "feature.hpp"
#include "keyFrame.hpp"
#include "mapPoint.hpp"

namespace lvio
{
  static std::size_t nextKeyFrameID = 0;

  KeyFrame::KeyFrame( std::shared_ptr<Frame> frame )
  {
    // std::cout << "KeyFrame::KeyFrame()" << std::endl;
    setKeyFrameId( nextKeyFrameID++ );
    setFrameId( frame->getFrameId() );
    setTimeStamp( frame->getTimeStamp() );
    setLeftImage( frame->getLeftImage() );
    setLeftFeatures( frame->getLeftFeatures() );
  }

  KeyFrame::Ptr KeyFrame::createKeyFramePtrFromFramPtr( std::shared_ptr<Frame> frame )
  {
    KeyFrame::Ptr key_frame_ptr = std::make_shared<KeyFrame>( frame );
    // std::cout << "Created KeyFrame.\n";
    std::size_t num_of_features = key_frame_ptr->getLeftFeatures().size();
    // std::cout << "Num: " << num_of_features << "\n";
    // feature 对应着 mapPoint，mapPoint 对应着 observations(一系列的feature)
    for ( std::size_t i = 0; i < num_of_features; i++ )
    {
      auto feature_ptr = key_frame_ptr->getLeftFeatures()[ i ];
      // std::cout << "Feature Ptr: " << feature_ptr << "\n";
      feature_ptr->setKeyFramePtr( key_frame_ptr );
      // std::cout << "KeyFrame Ptr: " << key_frame_ptr << "\n";
      auto map_point_ptr = feature_ptr->getMapPointPtr();
      if ( map_point_ptr )
      {
        map_point_ptr->addObservation( feature_ptr );
        key_frame_ptr->getLeftFeatures()[ i ]->setMapPointPtr( map_point_ptr );
      }
    }

    return key_frame_ptr;
  }

  void KeyFrame::setKeyFrameId( const std::size_t& key_frame_id )
  {
    m_key_frame_id = key_frame_id;
  }

  std::size_t KeyFrame::getKeyFrameId()
  {
    return m_key_frame_id;
  }

  void KeyFrame::setFrameId( const std::size_t& frame_id )
  {
    m_frame_id = frame_id;
  }

  std::size_t KeyFrame::getFrameId()
  {
    return m_frame_id;
  }

  void KeyFrame::setTimeStamp( const double& time_stamp )
  {
    m_time_stamp = time_stamp;
  }

  double KeyFrame::getTimeStamp()
  {
    return m_time_stamp;
  }

  void KeyFrame::setPose( const Sophus::SE3d& pose )
  {
    std::unique_lock<std::mutex> lock( update_pose_mutex_ );
    m_pose = pose;
  }

  Sophus::SE3d KeyFrame::getPose()
  {
    std::unique_lock<std::mutex> lock( update_pose_mutex_ );
    return m_pose;
  }

  void KeyFrame::setPoseToLastKeyFrame( const Sophus::SE3d& pose_to_last_key_frame )
  {
    m_pose_to_last_key_frame = pose_to_last_key_frame;
  }

  Sophus::SE3d KeyFrame::getPoseToLastKeyFrame()
  {
    return m_pose_to_last_key_frame;
  }

  void KeyFrame::setPoseToLoopKeyFrame( const Sophus::SE3d& pose_to_loop_key_frame )
  {
    m_pose_to_loop_key_frame = pose_to_loop_key_frame;
  }

  Sophus::SE3d KeyFrame::getPoseToLoopKeyFrame()
  {
    return m_pose_to_loop_key_frame;
  }

  std::vector<cv::KeyPoint> KeyFrame::getPyramidKeyPoints()
  {
    return m_key_points;
  }

  std::vector<cv::KeyPoint> KeyFrame::getKeyPoints()
  {
    std::size_t               num_of_features = getLeftFeatures().size();
    std::vector<cv::KeyPoint> key_points_( num_of_features );
    for ( std::size_t i = 0; i < num_of_features; i++ )
    {
      key_points_[ i ] = getLeftFeatures()[ i ]->getKeyPoint();
    }
    return key_points_;
  }

  void KeyFrame::setLeftFeatures( const std::vector<std::shared_ptr<Feature>>& features_in_left_image )
  {
    m_features_in_m_left_image = features_in_left_image;
  }

  std::vector<std::shared_ptr<Feature>> KeyFrame::getLeftFeatures()
  {
    return m_features_in_m_left_image;
  }

  void KeyFrame::setORBDescriptors( const cv::Mat& ORB_descriptors )
  {
    m_descriptors = ORB_descriptors;
  }

  cv::Mat KeyFrame::getORBDescriptors()
  {
    return m_descriptors;
  }

  void KeyFrame::setLeftImage( const cv::Mat& left_image )
  {
    m_left_image = left_image;
  }

  cv::Mat KeyFrame::getLeftImage()
  {
    return m_left_image;
  }

  void KeyFrame::setBowVec( const DBoW2::BowVector& bow_vec )
  {
    m_bow_vec = bow_vec;
  }

  DBoW2::BowVector KeyFrame::getBowVec()
  {
    return m_bow_vec;
  }

  void KeyFrame::setLastKeyFramePtr( std::shared_ptr<KeyFrame> last_key_frame )
  {
    m_wptr_last_key_frame = last_key_frame;
  }

  std::shared_ptr<KeyFrame> KeyFrame::getLastKeyFramePtr()
  {
    return m_wptr_last_key_frame.lock();
  }
  //
}  // namespace lvio