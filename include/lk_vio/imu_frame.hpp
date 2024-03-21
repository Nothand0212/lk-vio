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
#define EPSILON 1e-5

// single IMU data, including linear acceleration, angular velocity and timestamp
class IMUFrame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f a;  // linear acceleration in m/s^2
  Eigen::Vector3f w;  // angular velocity in rad/s
  double          t;  // timestamp in seconds
public:
  IMUFrame( const float &acc_x, const float &acc_y, const float &acc_z, const float &gyro_x, const float &gyro_y, const float &gyro_z, const double &timestamp )
      : a( acc_x, acc_y, acc_z ), w( gyro_x, gyro_y, gyro_z ), t( timestamp )
  {
  }
};

class IMUBias
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float bax, bay, baz;
  float bwx, bwy, bwz;

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
};

class IMUCalibration
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Sophus::SE3d                    T_imu_cam;  // IMU to camera transformation
  Sophus::SE3d                    T_cam_imu;  // Camera to IMU transformation
  Eigen::DiagonalMatrix<float, 6> cov, cov_walk;
  bool                            is_set;

public:
  IMUCalibration()
      : is_set{ false }
  {
  }

  IMUCalibration( const Sophus::SE3d &T_imu_cam, const float &ng, const float &na,
                  const float &ngw, const float &naw )
  {
    set( T_imu_cam, ng, na, ngw, naw );
  }

  IMUCalibration( const IMUCalibration &other )
  {
    this->is_set = other.is_set;

    this->T_imu_cam = other.T_imu_cam;
    this->T_cam_imu = other.T_cam_imu;
    this->cov       = other.cov;
    this->cov_walk  = other.cov_walk;
  }


  void set( const Sophus::SE3d &T_imu_cam, const float &ng, const float &na, const float &ngw, const float &naw )
  {
    this->is_set = true;

    float ng2  = ng * ng;
    float na2  = na * na;
    float ngw2 = ngw * ngw;
    float naw2 = naw * naw;

    this->T_imu_cam = T_imu_cam;
    this->T_cam_imu = T_imu_cam.inverse();

    this->cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    this->cov_walk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
  }
};

//Integration of 1 gyro measurement
class IntegratedRotation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3f delta_rotation;
  Eigen::Matrix3f right_jacobian;
  float           delta_time;

public:
  IntegratedRotation() = delete;
  IntegratedRotation( const Eigen::Vector3f &angle_velocity, const IMUBias &imu_bias, const float &delta_time )
  {
    const float x = ( angle_velocity.x() - imu_bias.bwx ) * delta_time;
    const float y = ( angle_velocity.y() - imu_bias.bwy ) * delta_time;
    const float z = ( angle_velocity.z() - imu_bias.bwz ) * delta_time;

    const float norm_squared = x * x + y * y + z * z;
    const float norm         = sqrt( norm_squared );

    Eigen::Vector3f rotation_vector;
    rotation_vector << x, y, z;

    Eigen::Matrix3f skew_symmetric_matrix = Sophus::SO3f::hat( rotation_vector );
    if ( norm < EPSILON )
    {
      delta_rotation = Eigen::Matrix3f::Identity() + skew_symmetric_matrix;
      right_jacobian = Eigen::Matrix3f::Identity();
    }
    else
    {
      delta_rotation = Eigen::Matrix3f::Identity() + skew_symmetric_matrix * sin( norm ) / norm + skew_symmetric_matrix * skew_symmetric_matrix * ( 1 - cos( norm ) ) / norm_squared;
      right_jacobian = Eigen::Matrix3f::Identity() + skew_symmetric_matrix * ( 1 - cos( norm ) ) / norm_squared + skew_symmetric_matrix * skew_symmetric_matrix * ( norm - sin( norm ) ) / ( norm * norm_squared );
    }
  }
};


}  // namespace lk_vio