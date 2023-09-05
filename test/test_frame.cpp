#include <gtest/gtest.h>

#include "frame.hpp"

TEST( FrameTest, Basic )
{
  // 创建一个 Frame 对象
  cv::Mat     left_img   = cv::imread( "/home/lin/Pictures/left.png" );
  cv::Mat     right_img  = cv::imread( "/home/lin/Pictures/right.png" );
  double      time_stamp = 0.0;
  lvio::Frame frame( left_img, right_img, time_stamp );

  // 测试帧的 id 和时间戳是否正确
  EXPECT_EQ( frame.getFrameId(), 0 );
  EXPECT_EQ( frame.getTimeStamp(), 0.0 );

  // 测试帧的左右图像是否正确
  EXPECT_EQ( frame.getLeftImage().rows, left_img.rows );
  EXPECT_EQ( frame.getLeftImage().cols, left_img.cols );
  EXPECT_EQ( frame.getRightImage().rows, right_img.rows );
  EXPECT_EQ( frame.getRightImage().cols, right_img.cols );

  // 测试帧的位姿是否正确
  Eigen::Matrix3d R_c_w = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_c_w = Eigen::Vector3d::Random();
  Sophus::SE3d    T_c_w( R_c_w, t_c_w );
  frame.setPose( T_c_w );
  EXPECT_TRUE( T_c_w.matrix().isApprox( frame.getPose().matrix() ) );

  // 测试帧的相对位姿是否正确
  Eigen::Matrix3d R_c_k = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_c_k = Eigen::Vector3d::Random();
  Sophus::SE3d    T_c_k( R_c_k, t_c_k );
  frame.setRelatitvePose( T_c_k );
  EXPECT_TRUE( T_c_k.matrix().isApprox( frame.getRelativePose().matrix() ) );

  // 测试帧的特征点是否正确
  std::vector<std::shared_ptr<lvio::Feature>> left_features;
  left_features.resize( 3 );
  frame.setLeftFeatures( left_features );
  EXPECT_EQ( frame.getLeftFeatures().size(), 3 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}