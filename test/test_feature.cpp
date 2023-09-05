#include <gtest/gtest.h>

#include "feature.hpp"

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

  // // 测试特征点的关键帧指针是否正确
  // std::shared_ptr<lvio::KeyFrame> key_frame = std::make_shared<lvio::KeyFrame>();
  // feature.setKeyFramePtr( key_frame );
  // EXPECT_EQ( feature.getKeyFramePtr(), key_frame );

  // // 测试特征点的地图点指针是否正确
  // std::shared_ptr<lvio::MapPoint> map_point = std::make_shared<lvio::MapPoint>();
  // feature.setMapPointPtr( map_point );
  // EXPECT_EQ( feature.getMapPointPtr(), map_point );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}