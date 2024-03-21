#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

namespace lk_vio
{
  // single IMU data, including linear acceleration, angular velocity and timestamp
  class IMUFrame
  {
  public:
    IMUFrame( const float &acc_x, const float &acc_y, const float &acc_z, const float &gyro_x, const float &gyro_y, const float &gyro_z, const double &timestamp )
        : a( acc_x, acc_y, acc_z ), w( gyro_x, gyro_y, gyro_z ), t( timestamp )
    {
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f a;  // linear acceleration in m/s^2
    Eigen::Vector3f w;  // angular velocity in rad/s
    double          t;  // timestamp in seconds
  };

  class IMUBias
  {
  public:
    IMUBias()
        : bax{ 0.0f }, bay{ 0.0f }, baz{ 0.0f }, bwx{ 0.0f }, bwy{ 0.0f }, bwz{ 0.0f }
    {
    }

    IMUBias( const float &bias_acc_x, const float &bias_acc_y, const float &bias_acc_z, const float &bias_gyro_x, const float &bias_gyro_y, const float &bias_gyro_z )
        : bax{ bias_acc_x }, bay{ bias_acc_y }, baz{ bias_acc_z }, bwx{ bias_gyro_x }, bwy{ bias_gyro_y }, bwz{ bias_gyro_z }
    {
    }

    void copyFrom( const IMUBias &other )
    {
      this->bax = other.bax;
      this->bay = other.bay;
      this->baz = other.baz;
      this->bwx = other.bwx;
      this->bwy = other.bwy;
      this->bwz = other.bwz;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float bax, bay, baz;
    float bwx, bwy, bwz;
  };

  class IMUCalibration
  {
  public:
    IMUCalibration()
        : is_set{ false }
    {
    }

    IMUCalibration( const Sophus::SE3d &t_imu_cam, const float &ng, const float &na,
                    const float &ngw, const float &naw )
    {
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sophus::SE3d                    t_imu_cam;  // IMU to camera transformation
    Sophus::SE3d                    t_cam_imu;  // Camera to IMU transformation
    Eigen::DiagonalMatrix<float, 6> cov, cov_walk;
    bool                            is_set;
  };
}  // namespace lk_vio