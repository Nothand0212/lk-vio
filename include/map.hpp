#ifndef LVIO_MAP_HPP_
#define LVIO_MAP_HPP_

#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace lvio
{
class Feature;
class MapPoint;
class KeyFrame;

class Map
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Map> Ptr;

  typedef std::unordered_map<unsigned long, std::shared_ptr<KeyFrame>> KeyFramesUnorderedMap;
  typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> MapPointsUnorderedMap;

  Map() = default;

  // @brief insert key frame
  void insertKeyFrame( std::shared_ptr<KeyFrame> key_frame );
  // @brief remove old active key frame
  void removeOldActiveKeyFrame();

  // @brief insert map point
  void insertMapPoint( std::shared_ptr<MapPoint> map_point );
  // @brief remove map point
  void removeMapPoint( std::shared_ptr<MapPoint> map_point );

  // @brief insert active map point
  void insertActiveMapPoint( std::shared_ptr<MapPoint> map_point );
  // @brief remove old active map point
  void removeOldActiveMapPoint();

  // @brief insert outlier map point's id, prepare to remove
  void addOutlierMapPoint( std::size_t map_point_id );
  // @brief remove all outlier map point
  void removeAllOutlierMapPoint();

  KeyFramesUnorderedMap getAllKeyFrames();
  KeyFramesUnorderedMap getActiveKeyFrames();
  MapPointsUnorderedMap getAllMapPoints();
  MapPointsUnorderedMap getActiveMapPoints();

public:
  //@brief 由外部调用
  std::mutex update_map_mutex_;

private:
  std::mutex update_outlier_map_points_;
  std::mutex update_data_mutex_;  // 内部调用

  MapPointsUnorderedMap all_map_points_;
  MapPointsUnorderedMap active_map_points_;
  KeyFramesUnorderedMap all_key_frames_;
  KeyFramesUnorderedMap active_key_frames_;

  std::list<std::size_t>    outlier_map_points_id_list;
  std::shared_ptr<KeyFrame> current_key_frame_     = nullptr;
  std::size_t               num_active_key_frames_ = 0;
};
//
}  // namespace lvio

#endif  // LVIO_MAP_HPP_