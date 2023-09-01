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

  void                      setKeyFrame( std::shared_ptr<KeyFrame> key_frame );
  std::shared_ptr<KeyFrame> getKeyFrame();

  void                      setMapPoint( std::shared_ptr<MapPoint> map_point );
  std::shared_ptr<MapPoint> getMapPoint();

  void setOutlierFlag( bool &is_outlier );
  bool getOutlierFlag();

  void setLeftImageFlag( bool &is_on_left_image );
  bool getLeftImageFlag();

  /* 变量 */
public:
  cv::KeyPoint key_point_;

private:
  std::weak_ptr<KeyFrame> key_frame_wptr_;
  std::weak_ptr<MapPoint> map_point_wprt_;

  bool is_outlier_       = false;
  bool is_on_left_image_ = true;  // 是否在左图像中
};
//
}  // namespace lvio

#endif  // LVIO_FEATURE_HPP