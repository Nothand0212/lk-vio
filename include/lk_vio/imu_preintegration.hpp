#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

#include "common/configuration.hpp"
#include "lk_vio/imu_frame.hpp"
namespace lk_vio
{
  constexpr float gravity_value = 9.81f;

  // Lie Algebra Functions
  Eigen::Matrix3d RightJacobianSO3( const float &x, const float &y, const float &z );
  Eigen::Matrix3d RightJacobianSO3( const Eigen::Vector3d &v );

  Eigen::Matrix3d InverseRightJacobianSO3( const float &x, const float &y, const float &z );
  Eigen::Matrix3d InverseRightJacobianSO3( const Eigen::Vector3d &v );

  Eigen::Matrix3d NormalizeRotation( const Eigen::Matrix3d &R );

  class IMUFrame;
  class IMUBias;
  class IMUCalibration;
  class IntegratedRotation;

  //Preintegration of Imu Measurements
  class IMUPreintegrator
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    float                            dT;
    Eigen::Matrix<double, 15, 15>    C;
    Eigen::Matrix<double, 15, 15>    Info;
    Eigen::DiagonalMatrix<double, 6> Nga, NgaWalk;

    // Values for the original bias (when integration was computed)
    IMUBias         b;
    Eigen::Matrix3d dR;
    Eigen::Vector3d dV, dP;
    Eigen::Matrix3d JRg, JVg, JVa, JPg, JPa;
    Eigen::Vector3d avgA, avgW;

  private:
    // Updated bias
    IMUBias bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    Eigen::Matrix<double, 6, 1> db;

    struct integrable
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      integrable() {}
      integrable( const Eigen::Vector3d &a_, const Eigen::Vector3d &w_, const float &t_ ) : a( a_ ), w( w_ ), t( t_ ) {}
      Eigen::Vector3d a, w;
      float           t;
    };

    std::vector<integrable> mvMeasurements;

    std::mutex mMutex;

  public:
    IMUPreintegrator( const IMUBias &b_, const IMUCalibration &calib );
    IMUPreintegrator( IMUPreintegrator *pImuPre );
    IMUPreintegrator() {}
    ~IMUPreintegrator() {}
    void    CopyFrom( IMUPreintegrator *pImuPre );
    void    Initialize( const IMUBias &b_ );
    void    IntegrateNewMeasurement( const Eigen::Vector3d &acceleration, const Eigen::Vector3d &angVel, const float &dt );
    void    Reintegrate();
    void    MergePrevious( IMUPreintegrator *pPrev );
    void    SetNewBias( const IMUBias &bu_ );
    IMUBias GetDeltaBias( const IMUBias &b_ );

    Eigen::Matrix3d GetDeltaRotation( const IMUBias &b_ );
    Eigen::Vector3d GetDeltaVelocity( const IMUBias &b_ );
    Eigen::Vector3d GetDeltaPosition( const IMUBias &b_ );

    Eigen::Matrix3d GetUpdatedDeltaRotation();
    Eigen::Vector3d GetUpdatedDeltaVelocity();
    Eigen::Vector3d GetUpdatedDeltaPosition();

    Eigen::Matrix3d GetOriginalDeltaRotation();
    Eigen::Vector3d GetOriginalDeltaVelocity();
    Eigen::Vector3d GetOriginalDeltaPosition();

    Eigen::Matrix<double, 6, 1> GetDeltaBias();

    IMUBias GetOriginalBias();
    IMUBias GetUpdatedBias();

    void printMeasurements() const
    {
      std::cout << "pint meas:\n";
      for ( int i = 0; i < mvMeasurements.size(); i++ )
      {
        std::cout << "meas " << mvMeasurements[ i ].t << std::endl;
      }
      std::cout << "end pint meas:\n";
    }
  };
}  // namespace lk_vio