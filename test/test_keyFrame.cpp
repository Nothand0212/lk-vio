#include <gtest/gtest.h>

#include "feature.hpp"
#include "frame.hpp"
#include "keyFrame.hpp"

TEST( KeyFrameTest, Basic )
{
  // 创建一个 Frame 对象
  cv::Mat          left_image  = cv::imread( "/home/lin/Pictures/left.png" );
  cv::Mat          right_image = cv::imread( "/home/lin/Pictures/right.png" );
  double           time_stamp  = 0.0;
  lvio::Frame::Ptr frame;
  frame.reset( new lvio::Frame( left_image, right_image, time_stamp ) );

  // 测试帧的特征点是否正确
  std::vector<std::shared_ptr<lvio::Feature>> left_features;

  cv::KeyPoint  key_point( 10, 20, 1.0 );
  lvio::Feature feature( key_point );
  auto          feature_ptr = std::make_shared<lvio::Feature>( feature );
  *feature_ptr              = feature;

  left_features.push_back( feature_ptr );
  frame->setLeftFeatures( left_features );

  // 创建一个 KeyFrame 对象
  std::shared_ptr<lvio::KeyFrame> key_frame = lvio::KeyFrame::createKeyFramePtrFromFramPtr( frame );

  // 测试关键帧的 ID 是否正确
  EXPECT_EQ( key_frame->getKeyFrameId(), 0 );

  // 测试关键帧的帧 ID 是否正确
  EXPECT_EQ( key_frame->getFrameId(), 0 );

  // 测试关键帧的时间戳是否正确
  EXPECT_EQ( key_frame->getTimeStamp(), 0.0 );

  // 测试关键帧的左图像是否正确
  EXPECT_EQ( cv::countNonZero( key_frame->getLeftImage() != left_image ), 0 );

  // 测试关键帧的特征点是否正确
  EXPECT_EQ( key_frame->getLeftFeatures().size(), 1 );

  // 测试关键帧的ORB描述符是否正确
  cv::Mat ORB_descriptors( 8, 32, CV_8UC1 );
  cv::randu( ORB_descriptors, cv::Scalar::all( 0 ), cv::Scalar::all( 255 ) );
  key_frame->setORBDescriptors( ORB_descriptors );
  EXPECT_EQ( cv::countNonZero( key_frame->getORBDescriptors() != ORB_descriptors ), 0 );

  // 测试关键帧的BoW向量是否正确
  DBoW2::BowVector bow_vec;
  key_frame->setBowVec( bow_vec );
  EXPECT_EQ( key_frame->getBowVec(), bow_vec );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}