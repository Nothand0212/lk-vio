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
  Eigen::Matrix3f RightJacobianSO3( const float &x, const float &y, const float &z );
  Eigen::Matrix3f RightJacobianSO3( const Eigen::Vector3f &v );

  Eigen::Matrix3f InverseRightJacobianSO3( const float &x, const float &y, const float &z );
  Eigen::Matrix3f InverseRightJacobianSO3( const Eigen::Vector3f &v );

  Eigen::Matrix3f NormalizeRotation( const Eigen::Matrix3f &R );

  class IMUFrame;
  class IMUBias;
  class IMUCalibration;
  class IntegratedRotation;

  //Preintegration of Imu Measurements
  class Preintegrated
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    float                           dT;
    Eigen::Matrix<float, 15, 15>    C;
    Eigen::Matrix<float, 15, 15>    Info;
    Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk;

    // Values for the original bias (when integration was computed)
    IMUBias         b;
    Eigen::Matrix3f dR;
    Eigen::Vector3f dV, dP;
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    Eigen::Vector3f avgA, avgW;

  private:
    // Updated bias
    IMUBias bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    Eigen::Matrix<float, 6, 1> db;

    struct integrable
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      integrable() {}
      integrable( const Eigen::Vector3f &a_, const Eigen::Vector3f &w_, const float &t_ ) : a( a_ ), w( w_ ), t( t_ ) {}
      Eigen::Vector3f a, w;
      float           t;
    };

    std::vector<integrable> mvMeasurements;

    std::mutex mMutex;

  public:
    Preintegrated( const IMUBias &b_, const IMUCalibration &calib );
    Preintegrated( Preintegrated *pImuPre );
    Preintegrated() {}
    ~Preintegrated() {}
    void    CopyFrom( Preintegrated *pImuPre );
    void    Initialize( const IMUBias &b_ );
    void    IntegrateNewMeasurement( const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt );
    void    Reintegrate();
    void    MergePrevious( Preintegrated *pPrev );
    void    SetNewBias( const IMUBias &bu_ );
    IMUBias GetDeltaBias( const IMUBias &b_ );

    Eigen::Matrix3f GetDeltaRotation( const IMUBias &b_ );
    Eigen::Vector3f GetDeltaVelocity( const IMUBias &b_ );
    Eigen::Vector3f GetDeltaPosition( const IMUBias &b_ );

    Eigen::Matrix3f GetUpdatedDeltaRotation();
    Eigen::Vector3f GetUpdatedDeltaVelocity();
    Eigen::Vector3f GetUpdatedDeltaPosition();

    Eigen::Matrix3f GetOriginalDeltaRotation();
    Eigen::Vector3f GetOriginalDeltaVelocity();
    Eigen::Vector3f GetOriginalDeltaPosition();

    Eigen::Matrix<float, 6, 1> GetDeltaBias();

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