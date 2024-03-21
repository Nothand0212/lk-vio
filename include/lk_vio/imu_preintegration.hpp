#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

#include "common/configuration.hpp"
namespace lk_vio
{
  constexpr float gravity_value = 9.81f;

  class IMUFrame
  {
  };

  class IMUPreintegration
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
}  // namespace lk_vio