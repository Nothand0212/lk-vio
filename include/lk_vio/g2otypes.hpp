#pragma once
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "Eigen/Core"
#include "sophus/se3.hpp"

namespace lk_vio

{
  class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

    /// left multiplication on SE3
    virtual void oplusImpl( const double *update ) override
    {
      Eigen::Matrix<double, 6, 1> update_eigen;
      update_eigen << update[ 0 ], update[ 1 ], update[ 2 ], update[ 3 ], update[ 4 ], update[ 5 ];
      _estimate = Sophus::SE3d::exp( update_eigen ) * _estimate;
    }

    virtual bool read( std::istream &in ) override { return true; }

    virtual bool write( std::ostream &out ) const override { return true; }
  };

  /// mappoint vertex
  class VertexXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override { _estimate = Eigen::Vector3d::Zero(); }

    virtual void oplusImpl( const double *update ) override
    {
      _estimate[ 0 ] += update[ 0 ];
      _estimate[ 1 ] += update[ 1 ];
      _estimate[ 2 ] += update[ 2 ];
    }

    virtual bool read( std::istream &in ) override { return true; }

    virtual bool write( std::ostream &out ) const override { return true; }
  };

  class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly( const Eigen::Vector3d &pos, const Eigen::Matrix3d &K )
        : _pos3d( pos ), _K( K )
    {
    }

    virtual void computeError() override
    {
      const VertexPose *v         = static_cast<VertexPose *>( _vertices[ 0 ] );
      Sophus::SE3d      T         = v->estimate();
      Eigen::Vector3d   pos_pixel = _K * ( T * _pos3d );
      pos_pixel /= pos_pixel[ 2 ];
      _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override
    {
      const VertexPose *v       = static_cast<VertexPose *>( _vertices[ 0 ] );
      Sophus::SE3d      T       = v->estimate();
      Eigen::Vector3d   pos_cam = T * _pos3d;
      double            fx      = _K( 0, 0 );
      double            fy      = _K( 1, 1 );
      double            X       = pos_cam[ 0 ];
      double            Y       = pos_cam[ 1 ];
      double            Z       = pos_cam[ 2 ];
      double            Zinv    = 1.0 / ( Z + 1e-18 );
      double            Zinv2   = Zinv * Zinv;
      _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
          -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
          fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;
    }

    virtual bool read( std::istream &in ) override { return true; }

    virtual bool write( std::ostream &out ) const override { return true; }

  private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
  };

  class EdgeProjection
      : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexXYZ>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection( const Eigen::Matrix3d &K, const Sophus::SE3d &cam_ext ) : _K( K )
    {
      _cam_ext = cam_ext;
    }

    virtual void computeError() override
    {
      const VertexPose *v0        = static_cast<VertexPose *>( _vertices[ 0 ] );
      const VertexXYZ * v1        = static_cast<VertexXYZ *>( _vertices[ 1 ] );
      Sophus::SE3d      T         = v0->estimate();
      Eigen::Vector3d   pos_pixel = _K * ( _cam_ext * ( T * v1->estimate() ) );
      pos_pixel /= pos_pixel[ 2 ];
      _error = _measurement - pos_pixel.head<2>();
    }

    //  virtual void linearizeOplus() override
    //  {
    //    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    //    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    //    Sophus::SE3d T = v0->estimate();
    //    Eigen::Vector3d pw = v1->estimate();
    //    Eigen::Vector3d pos_cam = _cam_ext * T * pw;
    //    double fx = _K(0, 0);
    //    double fy = _K(1, 1);
    //    double X = pos_cam[0];
    //    double Y = pos_cam[1];
    //    double Z = pos_cam[2];
    //    double Zinv = 1.0 / (Z + 1e-18);
    //    double Zinv2 = Zinv * Zinv;
    //    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
    //        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
    //        fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;
    //
    //    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * _cam_ext.rotationMatrix() *
    //                       T.rotationMatrix();
    //  }

    virtual bool read( std::istream &in ) override { return true; }

    virtual bool write( std::ostream &out ) const override { return true; }

  private:
    Eigen::Matrix3d _K;
    Sophus::SE3d    _cam_ext;
  };

  class EdgePoseGraph : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose, VertexPose>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError() override
    {
      const VertexPose *vertex0 = static_cast<VertexPose *>( _vertices[ 0 ] );
      const VertexPose *vertex1 = static_cast<VertexPose *>( _vertices[ 1 ] );
      Sophus::SE3d      v0      = vertex0->estimate();
      Sophus::SE3d      v1      = vertex1->estimate();
      _error                    = ( _measurement.inverse() * v0 * v1.inverse() ).log();
    }

    // virtual void linearizeOplus() override {
    //     // const VertexPose *vertex0 = static_cast<VertexPose *>(_vertices[0]);
    //     const VertexPose *vertex1 = static_cast<VertexPose *>(_vertices[1]);
    //     // SE3 v0 = vertex0->estimate();
    //     SE3 v1 = vertex1->estimate();

    //     Mat66 J;
    //     SE3 Error = SE3::exp(_error);
    //     J.block(0, 0, 3, 3) = Sophus::SO3d::hat(Error.so3().log());
    //     J.block(0, 3, 3, 3) = Sophus::SO3d::hat(Error.translation());
    //     J.block(3, 0, 3, 3) = Mat33::Zero(3, 3);
    //     J.block(3, 3, 3, 3) = Sophus::SO3d::hat(Error.so3().log());
    //     J = J * 0.5 + Mat66::Identity();

    //     _jacobianOplusXi = -J * v1.inverse().Adj();
    //     _jacobianOplusXj = J * v1.inverse().Adj();
    // }

    virtual bool read( std::istream &in ) override { return true; }

    virtual bool write( std::ostream &out ) const override { return true; }
  };

}  // namespace lk_vio
