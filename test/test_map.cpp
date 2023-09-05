#include <gtest/gtest.h>

#include "feature.hpp"
#include "frame.hpp"
#include "keyFrame.hpp"
#include "map.hpp"
#include "mapPoint.hpp"
TEST( MapTest, InsertKeyFrameTest )
{
  // 创建一个Map对象
  lvio::Map map;

  // 创建一个 Frame 对象
  cv::Mat          left_image  = cv::imread( "/home/lin/Pictures/left.png" );
  cv::Mat          right_image = cv::imread( "/home/lin/Pictures/right.png" );
  double           time_stamp  = 0.0;
  lvio::Frame::Ptr frame;
  frame.reset( new lvio::Frame( left_image, right_image, time_stamp ) );

  // 测试帧的特征点是否正确
  std::vector<std::shared_ptr<lvio::Feature>> left_features;

  cv::KeyPoint        key_point( 10, 20, 1.0 );
  lvio::Feature       feature( key_point );
  Eigen::Vector3d     position( 1.0, 2.0, 3.0 );
  lvio::MapPoint::Ptr map_point_ptr;
  map_point_ptr.reset( new lvio::MapPoint() );
  map_point_ptr->setPosition( position );
  feature.setMapPointPtr( map_point_ptr );
  auto feature_ptr = std::make_shared<lvio::Feature>( feature );

  left_features.push_back( feature_ptr );
  frame->setLeftFeatures( left_features );

  // 创建一个 KeyFrame 对象
  std::shared_ptr<lvio::KeyFrame> key_frame = lvio::KeyFrame::createKeyFramePtrFromFramPtr( frame );

  // 调用insertKeyFrame方法
  map.insertKeyFrame( key_frame );

  // 验证all_key_frames_和active_key_frames_中是否都包含了该关键帧
  EXPECT_TRUE( map.getAllKeyFrames().count( key_frame->getKeyFrameId() ) == 1 );
  EXPECT_TRUE( map.getActiveKeyFrames().count( key_frame->getKeyFrameId() ) == 1 );

  //   验证地图点的观测信息是否正确更新
  for ( auto &feature : key_frame->getLeftFeatures() )
  {
    auto map_point = feature->getMapPointPtr();
    // std::cout << "map point ptr: " << map_point.get() << std::endl;
    // std::cout << "feature ptr: " << feature.get() << std::endl;
    if ( map_point )
    {
      auto it = std::find_if( map_point->getActiveObservations().begin(), map_point->getActiveObservations().end(),
                              [ & ]( const std::weak_ptr<lvio::Feature> &weak_feature ) {
                                auto shared_feature = weak_feature.lock();
                                return shared_feature && shared_feature == feature;
                              } );
      EXPECT_TRUE( it != map_point->getActiveObservations().end() );
    }
  }
}

TEST( MapTest, RemoveMapPointTest )
{
  // 创建地图点
  Eigen::Vector3d     position_1( 1.0, 2.0, 3.0 );
  lvio::MapPoint::Ptr map_point_ptr_1;
  map_point_ptr_1.reset( new lvio::MapPoint() );
  map_point_ptr_1->setPosition( position_1 );
  //   std::cout << "map point 1 id: " << map_point_ptr_1->getId() << std::endl;

  Eigen::Vector3d     position_2( 4.0, 5.0, 6.0 );
  lvio::MapPoint::Ptr map_point_ptr_2;
  map_point_ptr_2.reset( new lvio::MapPoint() );
  map_point_ptr_2->setPosition( position_2 );
  //   std::cout << "map point 2 id: " << map_point_ptr_2->getId() << std::endl;
  // 将地图点插入到地图中
  lvio::Map map;
  map.insertMapPoint( map_point_ptr_1 );
  map.insertMapPoint( map_point_ptr_2 );
  map.insertActiveMapPoint( map_point_ptr_1 );
  map.insertActiveMapPoint( map_point_ptr_2 );

  // 删除地图点
  map.removeMapPoint( map_point_ptr_1 );
  // 检查地图中是否还存在该地图点
  auto all_map_points    = map.getAllMapPoints();
  auto active_map_points = map.getActiveMapPoints();
  EXPECT_EQ( all_map_points.count( 1 ), 0 );
  EXPECT_EQ( active_map_points.count( 1 ), 0 );
  EXPECT_EQ( all_map_points.count( 2 ), 1 );
  EXPECT_EQ( active_map_points.count( 2 ), 1 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}