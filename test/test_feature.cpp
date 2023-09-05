#include <gtest/gtest.h>

#include "feature.hpp"
#include "frame.hpp"
#include "keyFrame.hpp"
#include "mapPoint.hpp"

TEST( FeatureTest, Basic )
{
  // 创建一个关键点
  cv::KeyPoint key_point( 10, 20, 1.0 );

  // 创建一个特征点
  lvio::Feature feature( key_point );

  // 测试特征点的关键点是否正确
  EXPECT_EQ( feature.getKeyPoint().pt.x, 10 );
  EXPECT_EQ( feature.getKeyPoint().pt.y, 20 );
  EXPECT_EQ( feature.getKeyPoint().size, 1.0 );

  // 测试特征点的外点标志是否正确
  bool is_outlier = true;
  feature.setOutlierFlag( is_outlier );
  EXPECT_EQ( feature.getOutlierFlag(), is_outlier );

  // 测试特征点是否在左图像中
  bool is_on_left_image = true;
  feature.setLeftImageFlag( is_on_left_image );
  EXPECT_EQ( feature.getLeftImageFlag(), is_on_left_image );

  // 测试特征点的关键帧指针是否正确

  // 创建一个 Frame 对象
  cv::Mat          left_image  = cv::imread( "/home/lin/Pictures/left.png" );
  cv::Mat          right_image = cv::imread( "/home/lin/Pictures/right.png" );
  double           time_stamp  = 0.0;
  lvio::Frame::Ptr frame;
  frame.reset( new lvio::Frame( left_image, right_image, time_stamp ) );

  // 测试帧的特征点是否正确
  std::vector<std::shared_ptr<lvio::Feature>> left_features;

  auto feature_ptr = std::make_shared<lvio::Feature>( feature );
  *feature_ptr     = feature;

  left_features.push_back( feature_ptr );
  frame->setLeftFeatures( left_features );

  // 创建一个 KeyFrame 对象
  std::shared_ptr<lvio::KeyFrame> key_frame = lvio::KeyFrame::createKeyFramePtrFromFramPtr( frame );
  feature.setKeyFramePtr( key_frame );
  EXPECT_EQ( feature.getKeyFramePtr().get(), key_frame.get() );

  // 测试特征点的地图点指针是否正确
  Eigen::Vector3d     position( 1.0, 2.0, 3.0 );
  lvio::MapPoint::Ptr map_point_ptr;
  map_point_ptr.reset( new lvio::MapPoint() );
  map_point_ptr->setPosition( position );
  feature.setMapPointPtr( map_point_ptr );
  EXPECT_EQ( feature.getMapPointPtr().get(), map_point_ptr.get() );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}