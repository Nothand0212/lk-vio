#ifndef LVIO_KEYFRAME_HPP
#define LVIO_KEYFRAME_HPP

#include <Eigen/Core>
#include <vector>

#include "memory"
#include "mutex"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"

namespace lvio
{
class MapPoint;

class KeyFrame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<KeyFrame> Ptr;
  KeyFrame() = default;
  KeyFrame( std::shared_ptr<Frame> frame );
  ~KeyFrame() = default;
};
//
}  // namespace lvio

#endif  // LVIO_KEYFRAME_HPP