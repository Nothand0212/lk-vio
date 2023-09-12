#include "feature.hpp"

namespace lvio
{
Feature::Feature( const cv::KeyPoint &key_point )
{
  key_point_ = key_point;
}

Feature::Feature( std::shared_ptr<KeyFrame> key_frame, const cv::KeyPoint &key_point )
{
  setKeyFramePtr( key_frame );
  setKeyPoint( key_point );
}

void Feature::setKeyPoint( const cv::KeyPoint &key_point )
{
  key_point_ = key_point;
}

cv::KeyPoint Feature::getKeyPoint()
{
  return key_point_;
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
  is_on_left_image_ = is_on_left_image;
}

bool Feature::getLeftImageFlag()
{
  return is_on_left_image_;
}

void Feature::setKeyFramePtr( std::shared_ptr<KeyFrame> key_frame )
{
  key_frame_wptr_ = key_frame;
}

std::shared_ptr<KeyFrame> Feature::getKeyFramePtr()
{
  return key_frame_wptr_.lock();
}

void Feature::setMapPointPtr( std::shared_ptr<MapPoint> map_point )
{
  map_point_wprt_ = map_point;
}

std::shared_ptr<MapPoint> Feature::getMapPointPtr()
{
  return map_point_wprt_.lock();
}


//
}  // namespace lvio