#ifndef LVIO_G2O_TYPES_HPP
#define LVIO_G2O_TYPES_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "Eigen/Core"
#include "sophus/se3.hpp"

namespace lvio
{
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override
  {
    _estimate = Sophus::SE3d();  // _estimate 是 g2o::BaseVertex<6, Sophus::SE3d> 类的成员变量
  }

  virtual void oplusImpl( const double* update ) override
  {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[ 0 ], update[ 1 ], update[ 2 ], update[ 3 ], update[ 4 ], update[ 5 ];
    _estimate = Sophus::SE3d::exp( update_eigen ) * _estimate;
  }

  virtual bool read( std::istream& in ) override
  {
    return true;
  }

  virtual bool write( std::ostream& out ) const override
  {
    return true;
  }
};

// @brief 地图点的顶点
class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override
  {
    _estimate = Eigen::Vector3d();
  }

  virtual void oplusImpl( const double* update ) override
  {
    _estimate[ 0 ] += update[ 0 ];
    _estimate[ 1 ] += update[ 1 ];
    _estimate[ 2 ] += update[ 2 ];
  }

  virtual bool read( std::istream& in ) override
  {
    return true;
  }

  virtual bool write( std::ostream& out ) const override
  {
    return true;
  }
};

/**
 * @brief 一元边，观测值是二维向量(表示图像平面上的一个特征点的像素坐标)，连接的顶点是位姿。
 * 为了和g2o库的成员变量命名规则一致，位姿名为_pose，相机内参名为_K
 */
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjectionPoseOnly( const Eigen::Vector3d& pose, const Eigen::Matrix3d& K )
      : _pose3d( pose ), _K( K )
  {
  }

  virtual void computeError() override
  {
    const VertexPose* vertex_pose = static_cast<VertexPose*>( _vertices( 0 ) );

    Sophus::SE3d T = vertex_pose->estimate();

    Eigen::Vector3d pose_pixel = _K * ( T * _pose3d );
    pose_pixel /= pose_pixel[ 2 ];  // 将像素坐标归一化
    _error = _measurement - pose_pixel.head<2>();
  }

  virtual void linearizeOplus() override
  {
    const VertexPose* vertex_pose = static_cast<VertexPose*>( _vertices( 0 ) );
    Sophus::SE3d      T           = vertex_pose->estimate();
    Eigen::Vector3d   pose_camera = T * _pose3d;
    double            fx          = _K( 0, 0 );
    double            fy          = _K( 1, 1 );
    double            X           = pose_camera[ 0 ];
    double            Y           = pose_camera[ 1 ];
    double            Z           = pose_camera[ 2 ];
    double            Z_inv       = 1.0 / ( Z + 1e-18 );  // 防止除零
    double            Z_inv_2     = Z_inv * Z_inv;
    _jacobianOplusXi << -fx * Z_inv, 0, fx * X * Z_inv_2, fx * X * Y * Z_inv_2, -fx - fx * X * X * Z_inv_2,
        fx * Y * Z_inv, 0, -fy * Z_inv, fy * Y * Z_inv_2, fy + fy * Y * Y * Z_inv_2, -fy * X * Y * Z_inv_2,
        -fy * X * Z_inv;  // TODO check
  }

  virtual bool read( std::istream& in ) override
  {
    return true;
  }

  virtual bool write( std::ostream& out ) const override
  {
    return true;
  }

private:
  Eigen::Vector3d _pose3d;
  Eigen::Matrix3d _K;
  //
};

/**
 * @brief 二元边，观测值是二维向量(表示图像平面上的一个特征点的像素坐标)，连接的顶点是位姿和地图点。
 * 
 */
class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexPoint>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjection( const Eigen::Matrix3d& K, const Sophus::SE3d& camera_ext )
      : _K( K ), _camera_ext( camera_ext )
  {
  }

  virtual computeError() override
  {
    const VertexPose*  vertex_pose  = static_cast<VertexPose*>( _vertices( 0 ) );
    const VertexPoint* vertex_point = static_cast<VertexPoint*>( _vertices( 1 ) );

    Sophus::SE3d    T          = vertex_pose->estimate();
    Eigen::Vector3d pose_pixel = _K * ( _camera_ext * T * vertex_point->estimate() );

    pose_pixel /= pose_pixel[ 2 ];  // 归一化

    _error = _measurement - pose_pixel.head<2>();
  }

  virtual bool read( std::istream& in ) override
  {
    return true;
  }

  virtual bool write( std::ostream& out ) const override
  {
    return true;
  }

private:
  Eigen::Matrix3d _K;
  Sophus::SE3d    _camera_ext;
};

/**
 * @brief 二元边，观测值是欧氏变换(SE3d)，连接的顶点是两个位姿。
 * 
 */
class EdgePoseGraph : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void computeError() override
  {
    const VertexPose* vertex_pose_0 = static_cast<VertexPose*>( _vertices( 0 ) );
    const VertexPose* vertex_pose_1 = static_cast<VertexPose*>( _vertices( 1 ) );

    Sophus::SE3d T_0 = vertex_pose_0->estimate();
    Sophus::SE3d T_1 = vertex_pose_1->estimate();

    _error = ( _measurement.inverse() * T_0 * T_1.inverse() ).log();
  }

  virtual bool read( std::istream& in ) override
  {
    return true;
  }

  virtual bool write( std::ostream& out ) const override
  {
    return true;
  }
};
//
}  // namespace lvio


#endif  // LVIO_G2O_TYPES_HPP