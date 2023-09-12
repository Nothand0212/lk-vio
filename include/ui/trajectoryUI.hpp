#ifndef LVIO_TRAJECTORY_UI_HPP
#define LVIO_TRAJECTORY_UI_HPP

#include <utility>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "glog/logging.h"
#include "pangolin/gl/gl.h"
#include "pangolin/gl/glvbo.h"
#include "sophus/se3.hpp"

namespace ui
{
class TrajectoryUI
{
public:
  explicit TrajectoryUI( Eigen::Vector3d color )
      : color_( color )
  {
    CHECK( color_.size() == 3 );
  }

  void addTrajectoryPose( const Sophus::SE3d &pose );

  void render();

  void clear();

  std::vector<Sophus::SE3d> getTrajectoryPoses() const;
  {
    return pose_;
  }

private:
  std::size_t                  max_size_ = 1e6;
  std::vector<Eigen::Vector3f> positions_vec_;
  std::vector<Sopuhus::SE3f>   pose_vec_;
  Eigen::Vector3f              color_ = Eigen::Vector3f::Zero();
};
}  // namespace ui
#endif  // LVIO_TRAJECTORY_UI_HPP