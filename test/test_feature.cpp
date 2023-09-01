#include <gtest/gtest.h>

#include "feature.hpp"

TEST( FeatureTest, ConstructorTest )
{
  // 测试默认构造函数
  lvio::Feature feature1;
  EXPECT_EQ( feature1.getOutlierFlag(), false );
  EXPECT_EQ( feature1.getLeftImageFlag(), true );
  EXPECT_EQ( feature1.getKeyFrame(), nullptr );
  EXPECT_EQ( feature1.getMapPoint(), nullptr );

  // 测试带参数的构造函数
  cv::KeyPoint  key_point( 10, 20, 1.0 );
  lvio::Feature feature2( key_point );
  EXPECT_EQ( feature2.getOutlierFlag(), false );
  EXPECT_EQ( feature2.getLeftImageFlag(), true );
  EXPECT_EQ( feature2.getKeyFrame(), nullptr );
  EXPECT_EQ( feature2.getMapPoint(), nullptr );

  // 测试带参数的构造函数
  std::shared_ptr<lvio::KeyFrame> key_frame( new lvio::KeyFrame() );
  lvio::Feature                   feature3( key_frame, key_point );
  EXPECT_EQ( feature3.getOutlierFlag(), false );
  EXPECT_EQ( feature3.getLeftImageFlag(), true );
  EXPECT_EQ( feature3.getKeyFrame(), key_frame );
  EXPECT_EQ( feature3.getMapPoint(), nullptr );
}

TEST( FeatureTest, OutlierFlagTest )
{
  lvio::Feature feature;
  EXPECT_EQ( feature.getOutlierFlag(), false );

  feature.setOutlierFlag( true );
  EXPECT_EQ( feature.getOutlierFlag(), true );

  feature.setOutlierFlag( false );
  EXPECT_EQ( feature.getOutlierFlag(), false );
}

TEST( FeatureTest, LeftImageFlagTest )
{
  lvio::Feature feature;
  EXPECT_EQ( feature.getLeftImageFlag(), true );

  feature.setLeftImageFlag( false );
  EXPECT_EQ( feature.getLeftImageFlag(), false );

  feature.setLeftImageFlag( true );
  EXPECT_EQ( feature.getLeftImageFlag(), true );
}

// TEST( FeatureTest, KeyFrameTest )
// {
//   lvio::Feature feature;
//   EXPECT_EQ( feature.getKeyFrame(), nullptr );

//   std::shared_ptr<lvio::KeyFrame> key_frame( new lvio::KeyFrame() );
//   feature.setKeyFrame( key_frame );
//   EXPECT_EQ( feature.getKeyFrame(), key_frame );

//   feature.setKeyFrame( nullptr );
//   EXPECT_EQ( feature.getKeyFrame(), nullptr );
// }

// TEST( FeatureTest, MapPointTest )
// {
//   lvio::Feature feature;
//   EXPECT_EQ( feature.getMapPoint(), nullptr );

//   std::shared_ptr<lvio::MapPoint> map_point( new lvio::MapPoint() );
//   feature.setMapPoint( map_point );
//   EXPECT_EQ( feature.getMapPoint(), map_point );

//   feature.setMapPoint( nullptr );
//   EXPECT_EQ( feature.getMapPoint(), nullptr );
// }

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}