#include "lk_vio/frame.hpp"

namespace lk_vio
{
  /// 0~4294967295 0-2^32
  static unsigned long FrmaeFactoryId = 0;

  Frame::Frame( const cv::Mat &left_image, const cv::Mat &right_image, const double &timestamp )
      : left_image_( left_image ), right_image_( right_image ), timestamp_( timestamp )
  {
    // left_image_  = left_image;
    // right_image_ = right_image;
    // timestamp_   = timestamp;

    frame_id_ = FrmaeFactoryId++;
  }

  Frame::~Frame()
  {
    // delete last_frame_;
    delete imu_preintegration_kf_;
    delete imu_preintegration_lf_;

    std::cout << "Frame " << frame_id_ << " is deleted." << std::endl;
  }

  Sophus::SE3d Frame::getPose()
  {
    std::unique_lock<std::mutex> lck( update_pose_ );
    return pose_;
  }

  void Frame::SetPose( const Sophus::SE3d &pose )
  {
    std::unique_lock<std::mutex> lck( update_pose_ );
    pose_ = pose;
  }

  Sophus::SE3d Frame::getRelativePose()
  {
    std::unique_lock<std::mutex> lck( update_realteive_pose_ );
    return relative_pose_to_kf_;
  }

  void Frame::SetRelativePose( const Sophus::SE3d &relative_pose )
  {
    std::unique_lock<std::mutex> lck( update_realteive_pose_ );
    relative_pose_to_kf_ = relative_pose;
  }

  // related with IMU preintegration

  void Frame::setIntegratedFlag()
  {
    this->is_intigrated_.store( true );
  }

  bool Frame::getIntegratedFlag()
  {
    return this->is_intigrated_.load();
  }

  void Frame::setLastFramePtr( std::shared_ptr<Frame> last_frame )
  {
    this->last_frame_sptr_ = last_frame;
  }

  std::shared_ptr<Frame> Frame::getLastFramePtr()
  {
    return this->last_frame_sptr_;
  }

}  // namespace lk_vio