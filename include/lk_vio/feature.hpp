#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include "memory"
#include "opencv2/opencv.hpp"


namespace lk_vio
{
  class MapPoint;
  class KeyFrame;
  class Feature
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;
    Feature() = default;
    Feature( const cv::KeyPoint &kp );
    Feature( std::shared_ptr<KeyFrame> kf, const cv::KeyPoint &kp );

  public:
    std::weak_ptr<KeyFrame>   keyframe_;
    cv::KeyPoint              kp_position_;
    std::vector<cv::KeyPoint> pyramid_keypoints_;
    std::weak_ptr<MapPoint>   map_point_;

    bool is_on_left_frame_ = true;  // true: on left frame; false: on right frame;
    bool is_outlier_       = false;
  };

}  // namespace lk_vio
