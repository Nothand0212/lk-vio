#include <gtest/gtest.h>

#include "feature.hpp"
#include "mapPoint.hpp"

TEST( MapPointTest, Basic )
{
  // 创建一个 MapPoint 对象
  Eigen::Vector3d position( 1.0, 2.0, 3.0 );
  lvio::MapPoint  map_point;
  map_point.setPosition( position );

  // 测试地图点的 id 和位置是否正确
  EXPECT_EQ( map_point.getId(), 0 );
  EXPECT_TRUE( position.isApprox( map_point.getPosition() ) );

  // 测试添加观测到该地图点的特征点
  std::shared_ptr<lvio::Feature> feature1( new lvio::Feature() );
  std::shared_ptr<lvio::Feature> feature2( new lvio::Feature() );
  map_point.addObservation( feature1 );
  map_point.addObservation( feature2 );
  EXPECT_EQ( map_point.getObservations().size(), 2 );
  EXPECT_EQ( map_point.getObservedTimes(), 2 );

  // 测试删除观测到该地图点的特征点
  map_point.removeObservation( feature1 );
  EXPECT_EQ( map_point.getObservations().size(), 1 );
  EXPECT_EQ( map_point.getObservedTimes(), 1 );

  // 测试添加观测到该地图点的激活特征点
  map_point.addActiveObservation( feature1 );
  map_point.addActiveObservation( feature2 );
  EXPECT_EQ( map_point.getActiveObservations().size(), 2 );
  EXPECT_EQ( map_point.getActiveObservedTimes(), 2 );

  // 测试删除观测到该地图点的激活特征点
  map_point.removeActiveObservation( feature1 );
  EXPECT_EQ( map_point.getActiveObservations().size(), 1 );
  EXPECT_EQ( map_point.getActiveObservedTimes(), 1 );

  // 测试MapPoint的ID
  lvio::MapPoint map_point_new;
  EXPECT_EQ( map_point_new.getId(), 1 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}