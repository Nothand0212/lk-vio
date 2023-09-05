/**
 * @file map.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-09-05
 * 
 * @copyright Copyright (c) 2023
 * @par History:
 * 
 */

#include "feature.hpp"
#include "keyFrame.hpp"
#include "map.hpp"
#include "mapPoint.hpp"
#include "utility.hpp"

namespace lvio
{
Map::Map()
{
  // TODO this should be a parameter, will be integrated with ROS later
  num_active_key_frames_ = 100;
}

void Map::insertKeyFrame( std::shared_ptr<KeyFrame> key_frame )
{
  current_key_frame_ = key_frame;

  {
    std::unique_lock<std::mutex> lock( update_data_mutex_ );
    // 在all_key_frames_中搜索是否已经存在该关键帧，没有则插入
    if ( all_key_frames_.find( key_frame->getKeyFrameId() ) == all_key_frames_.end() )
    {
      all_key_frames_.insert( std::make_pair( key_frame->getKeyFrameId(), key_frame ) );
      active_key_frames_.insert( std::make_pair( key_frame->getKeyFrameId(), key_frame ) );
    }
    else
    {
      LOG( ERROR ) << "This Key Frame Has Been Inserted into the Map. Update it.";
      all_key_frames_[ key_frame->getKeyFrameId() ]    = key_frame;
      active_key_frames_[ key_frame->getKeyFrameId() ] = key_frame;
    }
  }

  // 由于添加了新的关键帧，所以需要更新地图点的观测信息
  for ( auto &feature : key_frame->getLeftFeatures() )
  {
    auto map_point = feature->getMapPointPtr();
    if ( map_point )
    {
      map_point->addActiveObservation( feature );
      insertActiveMapPoint( map_point );
    }
  }

  // 如果关键帧数量超过了阈值，则删除旧的关键帧
  if ( active_key_frames_.size() > num_active_key_frames_ )
  {
    removeOldActiveKeyFrame();
    removeOldActiveMapPoint();
  }
}

void Map::insertMapPoint( MapPoint::Ptr map_point )
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  if ( all_map_points_.find( map_point->getId() ) == all_map_points_.end() )
  {
    all_map_points_.insert( std::make_pair( map_point->getId(), map_point ) );
  }
  else
  {
    LOG( FATAL ) << "This Map Point Has Been Inserted into the Map Points Unordered Map. Update it.";
    all_map_points_[ map_point->getId() ] = map_point;
  }
}

void Map::insertActiveMapPoint( MapPoint::Ptr map_point )
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  if ( active_map_points_.find( map_point->getId() ) == active_map_points_.end() )
  {
    active_map_points_.insert( std::make_pair( map_point->getId(), map_point ) );
  }
  else
  {
    LOG( FATAL ) << "This Map Point Has Been Inserted into the Active Map Points Unordered Map. Update it.";
    active_map_points_[ map_point->getId() ] = map_point;
  }
}

void Map::removeOldActiveKeyFrame()
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  if ( current_key_frame_ == nullptr )
  {
    LOG( WARNING ) << "Current Key Frame is Null.";
    return;
  }

  double max_distance     = 0;
  double min_distance     = 9999999;
  double max_key_frame_id = 0;
  double min_key_frame_id = 0;

  // 遍历所有的关键帧，找到距离当前关键帧最远的关键帧
  auto T_w_c = current_key_frame_->getPose().inverse();

  for ( auto &key_frame_pair : active_key_frames_ )
  {
    if ( key_frame_pair.second == current_key_frame_ )
    {
      continue;
    }

    double distance = ( key_frame_pair.second->getPose() * T_w_c ).log().norm();

    if ( distance > max_distance )
    {
      max_distance     = distance;
      max_key_frame_id = key_frame_pair.first;
    }
    else if ( distance < min_distance )
    {
      min_distance     = distance;
      min_key_frame_id = key_frame_pair.first;
    }
  }

  // 选择的参考关键帧最好是有点距离，但又不能太远距离
  const float   min_distance_threshold = 0.2;  // TODO 设为参数
  KeyFrame::Ptr frame_to_remove        = nullptr;
  if ( min_distance < min_distance_threshold )
  {
    frame_to_remove = all_key_frames_[ min_key_frame_id ];
  }
  else
  {
    frame_to_remove = all_key_frames_[ max_key_frame_id ];
  }

  // 将该关键帧从active_key_frames_中删除，并删除地图点对应的观测
  active_key_frames_.erase( frame_to_remove->getKeyFrameId() );
  for ( auto &feature : frame_to_remove->getLeftFeatures() )
  {
    auto map_point = feature->getMapPointPtr();
    if ( map_point )
    {
      map_point->removeObservation( feature );
    }
  }
}

void Map::removeOldActiveMapPoint()
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  for ( auto iteration = active_map_points_.begin(); iteration != active_map_points_.end(); )
  {
    if ( iteration->second->getObservedTimes() == 0 )
    {
      iteration = active_map_points_.erase( iteration );  // erase() 返回下一个元素的迭代器
    }
    else
    {
      ++iteration;
    }
  }
}

void Map::removeMapPoint( MapPoint::Ptr map_point )
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  std::size_t map_point_id = map_point->getId();
  all_map_points_.erase( map_point_id );
  active_map_points_.erase( map_point_id );
}

void Map::addOutlierMapPoint( std::size_t map_point_id )
{
  std::unique_lock<std::mutex> lock( update_outlier_map_points_ );

  outlier_map_points_id_list.push_back( map_point_id );
}

void Map::removeAllOutlierMapPoint()
{
  std::unique_lock<std::mutex> lock( update_outlier_map_points_ );
  std::unique_lock<std::mutex> lock2( update_data_mutex_ );

  for ( auto iteration = outlier_map_points_id_list.begin(); iteration != outlier_map_points_id_list.end(); iteration++ )
  {
    all_map_points_.erase( *iteration );
    active_map_points_.erase( *iteration );
  }
  outlier_map_points_id_list.clear();
}

Map::MapPointsUnorderedMap Map::getAllMapPoints()
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  return all_map_points_;
}

Map::MapPointsUnorderedMap Map::getActiveMapPoints()
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  return active_map_points_;
}

Map::KeyFramesUnorderedMap Map::getAllKeyFrames()
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  return all_key_frames_;
}

Map::KeyFramesUnorderedMap Map::getActiveKeyFrames()
{
  std::unique_lock<std::mutex> lock( update_data_mutex_ );

  return active_key_frames_;
}
//
}  // namespace lvio