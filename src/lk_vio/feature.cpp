#include "lk_vio/feature.hpp"

namespace lk_vio
{
  Feature::Feature( std::shared_ptr<KeyFrame> kf, const cv::KeyPoint &kp )
  {
    keyframe_    = kf;
    kp_position_ = kp;
  }

  Feature::Feature( const cv::KeyPoint &kp )
  {
    kp_position_ = kp;
  }

}  // namespace lk_vio