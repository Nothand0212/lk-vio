#ifndef LVIO_FEATURE_HPP
#define LVIO_FEATURE_HPP

#include <Eigen/Core>
#include <vector>

#include "memory"
#include "mutex"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"


namespace lvio
{
class MapPoint;
class KeyFrame;

class Feature
{
  /* 方法 */
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重命名智能指针
  typedef std::shared_ptr<Feature> Ptr;
  Feature() = default;
  Feature( const cv::KeyPoint &key_point );
  Feature( std::shared_ptr<KeyFrame> key_frame, const cv::KeyPoint &key_point );
  ~Feature() = default;

  void         setKeyPoint( const cv::KeyPoint &key_point );
  cv::KeyPoint getKeyPoint();

  void                      setKeyFramePtr( std::shared_ptr<KeyFrame> key_frame );
  std::shared_ptr<KeyFrame> getKeyFramePtr();

  void                      setMapPointPtr( std::shared_ptr<MapPoint> map_point );
  std::shared_ptr<MapPoint> getMapPointPtr();

  void setOutlierFlag( bool &is_outlier );
  bool getOutlierFlag();

  void setLeftImageFlag( bool &is_on_left_image );
  bool getLeftImageFlag();


private:
  cv::KeyPoint            key_point_;
  std::weak_ptr<KeyFrame> key_frame_wptr_;
  std::weak_ptr<MapPoint> map_point_wprt_;

  bool is_outlier_       = false;
  bool is_on_left_image_ = true;  // 是否在左图像中
};
//
}  // namespace lvio

#endif  // LVIO_FEATURE_HPP