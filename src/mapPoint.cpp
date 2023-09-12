#include "feature.hpp"
#include "mapPoint.hpp"

namespace lvio
{
static size_t nextMapPointId = 0;

// @brief: 无参构造函数
MapPoint::MapPoint()
{
  setId( nextMapPointId++ );
}

// /**
//  * @brief Construct a new Map Point:: Map Point object
//  * 不要用这种构造方法，会导致地图点的id不连续
//  * @param    id        地图点的id
//  * @param    position  地图点的位置(xyz)
//  */
// MapPoint::MapPoint( const size_t &id, const Eigen::Vector3d &position )
// {
//   setId( id );
//   setPosition( position );
// }

// @brief: 设置地图点的id
void MapPoint::setId( const size_t &id )
{
  id_ = id;
}

// @brief: 获取地图点的id
size_t MapPoint::getId()
{
  return id_;
}

//@brief: 获取地图点的观测次数
int MapPoint::getObservedTimes()
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  return observed_times_;
}

// @brief: 获取地图点的激活观测次数
int MapPoint::getActiveObservedTimes()
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  return active_observed_times_;
}

// @brief: 设置地图点的位置(xyz)
void MapPoint::setPosition( const Eigen::Vector3d &position )
{
  std::unique_lock<std::mutex> lock( update_position_mutex_ );
  position_ = position;
}

// @brief: 获取地图点的位置(xyz)
Eigen::Vector3d MapPoint::getPosition()
{
  std::unique_lock<std::mutex> lock( update_position_mutex_ );
  return position_;
}

/**
 * @brief  添加观测到该地图点的特征点
 * 
 * @param    feature   特征点，智能指针
 */
void MapPoint::addObservation( std::shared_ptr<Feature> feature )
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  observations_.push_back( feature );
  observed_times_++;
}

// @brief 添加观测到该地图点的特征点
void MapPoint::addActiveObservation( std::shared_ptr<Feature> feature )
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  active_observations_.push_back( feature );
  active_observed_times_++;
}

/**
 * @brief 删除观测到该地图点的特征点
 * 由于都是智能指针，直接判断指针指向的地址是否相同即可，相同则删除
 * @param    feature   特征点，智能指针
 */
void MapPoint::removeObservation( std::shared_ptr<Feature> feature )
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  for ( auto iter = observations_.begin(); iter != observations_.end(); iter++ )
  {
    if ( iter->lock() == feature )
    {
      observations_.erase( iter );        // 删除观测到该地图点的特征点
      feature->getMapPointPtr().reset();  // 删除该特征点观测到的地图点
      observed_times_--;
      break;
    }
  }
}

// @brief 删除观测到该地图点的特征点
void MapPoint::removeActiveObservation( std::shared_ptr<Feature> feature )
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  for ( auto iter = active_observations_.begin(); iter != active_observations_.end(); iter++ )
  {
    if ( iter->lock() == feature )
    {
      active_observations_.erase( iter );  // 删除观测到该地图点的特征点
      feature->getMapPointPtr().reset();   // 删除该特征点观测到的地图点
      active_observed_times_--;
      break;
    }
  }
}

// @brief 获取观测到该地图点的所有特征点(list)
std::list<std::weak_ptr<Feature>> MapPoint::getObservations()
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  return observations_;
}

// @brief 获取观测到该地图点的所有特征点(list)
std::list<std::weak_ptr<Feature>> MapPoint::getActiveObservations()
{
  std::unique_lock<std::mutex> lock( update_observation_mutex_ );
  return active_observations_;
}

// @brief 设置地图点的外点标志位
void MapPoint::setOutlierFlag( bool is_outlier )
{
  is_outlier_ = is_outlier;
}

// @brief 获取地图点的外点标志位
bool MapPoint::getOutlierFlag()
{
  return is_outlier_;
}

//
}  // namespace lvio