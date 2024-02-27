#ifndef LVIO_UTILITY_HPP
#define LVIO_UTILITY_HPP

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace lvio
{
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXXd;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1>              VecXd;
  typedef Eigen::Matrix<double, 3, 4>                           Mat34d;

  // static std::once_flag singleton_flag;

  class Utility
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //
  };
  //

  /**
 * @brief 利用SVD计算三角化空间点
 * 
 * @param    pose_vec     位姿vector，左右目相机位姿
 * @param    point_vec    平面点vector，左右目相机观测到的平面点
 * @param    point_world  三角化后的空间点
 * @return true 求解成功
 * @return false 
 */
  inline bool traiangulation( const std::vector<Sophus::SE3d> &pose_vec, const std::vector<Eigen::Vector3d> &point_vec, Eigen::Vector3d &point_world )
  {
    MatXXd A( 2 * pose_vec.size(), 4 );
    VecXd  b( 2 * pose_vec.size() );
    for ( size_t i = 0; i < pose_vec.size(); ++i )
    {
      Mat34d mat                    = pose_vec[ i ].matrix3x4();
      A.block<1, 4>( 2 * i, 0 )     = point_vec[ i ][ 0 ] * mat.row( 2 ) - mat.row( 0 );
      A.block<1, 4>( 2 * i + 1, 0 ) = point_vec[ i ][ 1 ] * mat.row( 2 ) - mat.row( 1 );
    }
    auto svd    = A.bdcSvd( Eigen::ComputeThinU | Eigen::ComputeThinV );
    point_world = ( svd.matrixV().col( 3 ) / svd.matrixV()( 3, 3 ) ).head<3>();

    if ( svd.singularValues()[ 3 ] / svd.singularValues()[ 2 ] < 1e-2 )
    {
      return true;
    }

    return false;
  }
}  // namespace lvio
#endif  // LVIO_UTILITY_HPP