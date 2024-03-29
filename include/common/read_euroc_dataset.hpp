#pragma once

#include <Eigen/Core>
#include <vector>

namespace common
{
  void read_euroc_dataset( const std::string &           str_path_to_sequence,
                           std::vector<std::string> &    str_image_left_vec_path,
                           std::vector<std::string> &    str_image_right_vec_path,
                           std::vector<Eigen::Vector3d> &acc_vec,
                           std::vector<Eigen::Vector3d> &gyro_vec,
                           std::vector<double> &         timestamps_vec )
  {
    // TODO: Implement this function
  }
}  // namespace common