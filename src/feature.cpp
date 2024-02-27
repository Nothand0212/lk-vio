#include "feature.hpp"

namespace lvio
{
  Feature::Feature( const cv::KeyPoint &key_point )
  {
    m_key_point = key_point;
  }

  Feature::Feature( std::shared_ptr<KeyFrame> key_frame, const cv::KeyPoint &key_point )
  {
    setKeyFramePtr( key_frame );
    setKeyPoint( key_point );
  }

  void Feature::setKeyPoint( const cv::KeyPoint &key_point )
  {
    m_key_point = key_point;
  }

  cv::KeyPoint Feature::getKeyPoint()
  {
    return m_key_point;
  }

  void Feature::setOutlierFlag( bool is_outlier )
  {
    is_outlier_ = is_outlier;
  }

  bool Feature::getOutlierFlag()
  {
    return is_outlier_;
  }

  void Feature::setLeftImageFlag( bool is_on_left_image )
  {
    is_on_m_left_image = is_on_left_image;
  }

  bool Feature::getLeftImageFlag()
  {
    return is_on_m_left_image;
  }

  void Feature::setKeyFramePtr( std::shared_ptr<KeyFrame> key_frame )
  {
    m_wptr_key_frame = key_frame;
  }

  std::shared_ptr<KeyFrame> Feature::getKeyFramePtr()
  {
    return m_wptr_key_frame.lock();
  }

  void Feature::setMapPointPtr( std::shared_ptr<MapPoint> map_point )
  {
    m_wptr_map_point = map_point;
  }

  std::shared_ptr<MapPoint> Feature::getMapPointPtr()
  {
    return m_wptr_map_point.lock();
  }


  //
}  // namespace lvio