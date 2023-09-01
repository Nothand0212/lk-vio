#include "frame.hpp"

namespace lvio
{
static std::size_t nextFrameId = 0;

Frame::Frame( const cv::Mat &left_img, const cv::Mat &right_img, const double &time_stamp )
{
  setLeftImage( left_img );
  setRightImage( right_img );
  setTimeStamp( time_stamp );
  setFrameId( nextFrameId++ );
}

void Frame::setPose( const Sophus::SE3d &T_c_w )
{
  std::unique_lock<std::mutex> lock( updating_pose_mutex_ );
  T_c_w_ = T_c_w;
}

Sophus::SE3d Frame::getPose()
{
  std::unique_lock<std::mutex> lock( updating_pose_mutex_ );
  return T_c_w_;
}

void Frame::setRelatitvePose( const Sophus::SE3d &T_c_k )
{
  std::unique_lock<std::mutex> lock( updating_relative_pose_mutex_ );
  T_c_k_ = T_c_k;
}

Sophus::SE3d Frame::getRelativePose()
{
  std::unique_lock<std::mutex> lock( updating_relative_pose_mutex_ );
  return T_c_k_;
}

void Frame::setTimeStamp( const double &time_stamp )
{
  time_stamp_ = time_stamp;
}

double Frame::getTimeStamp()
{
  return time_stamp_;
}

void Frame::setFrameId( const std::size_t &frame_id )
{
  frame_id_ = frame_id;
}

std::size_t Frame::getFrameId()
{
  return frame_id_;
}

void Frame::setLeftImage( const cv::Mat &left_img )
{
  left_img_ = left_img;
}

cv::Mat Frame::getLeftImage()
{
  return left_img_;
}

void Frame::setRightImage( const cv::Mat &right_img )
{
  right_img_ = right_img;
}

cv::Mat Frame::getRightImage()
{
  return right_img_;
}

void Frame::setLeftFeatures( const std::vector<std::shared_ptr<Feature>> &left_features )
{
  left_features_ = left_features;
}

std::vector<std::shared_ptr<Feature>> Frame::getLeftFeatures()
{
  return left_features_;
}

void Frame::setRightFeatures( const std::vector<std::shared_ptr<Feature>> &right_features )
{
  right_features_ = right_features;
}

std::vector<std::shared_ptr<Feature>> Frame::getRightFeatures()
{
  return right_features_;
}

//
}  // namespace lvio