#include "camera.hpp"

namespace lvio
{
  Eigen::Vector3d Camera::worldToCamera( const Eigen::Vector3d &point_world, const Sophus::SE3d &T_c_w )
  {
    return m_pose * T_c_w * point_world;
  }

  Eigen::Vector3d Camera::cameraToWorld( const Eigen::Vector3d &point_camera, const Sophus::SE3d &T_c_w )
  {
    return T_c_w.inverse() * pose_inv_ * point_camera;
  }
  /**
 * @brief 计算像素坐标
 * 接受一个相机坐标系下的点，通过相机的内参，计算出像素坐标
 * @param    point_camera   相机坐标系下的点
 * @return Eigen::Vector2d  像素坐标
 */
  Eigen::Vector2d Camera::cameraToPixel( const Eigen::Vector3d &point_camera )
  {
    return Eigen::Vector2d( fx_ * point_camera( 0, 0 ) / point_camera( 2, 0 ) + cx_,
                            fy_ * point_camera( 1, 0 ) / point_camera( 2, 0 ) + cy_ );
  }

  Eigen::Vector3d Camera::pixelToCamera( const Eigen::Vector2d &point_pixel, const double &depth )
  {
    return Eigen::Vector3d( ( point_pixel( 0, 0 ) - cx_ ) * depth / fx_,
                            ( point_pixel( 1, 0 ) - cy_ ) * depth / fy_, depth );
  }

  Eigen::Vector3d Camera::pixelToWorld( const Eigen::Vector2d &point_pixel, const Sophus::SE3d &T_c_w, const double &depth )
  {
    return cameraToWorld( pixelToCamera( point_pixel, depth ), T_c_w );
  }

  Eigen::Vector2d Camera::worldToPixel( const Eigen::Vector3d &point_world, const Sophus::SE3d &T_c_w )
  {
    return cameraToPixel( worldToCamera( point_world, T_c_w ) );
  }

  void Camera::unDistortImage( cv::Mat &src, cv::Mat &dst )
  {
    cv::Mat distorted_image = src.clone();
    cv::Mat K_cv            = cv::Mat::zeros( 3, 3, CV_32F );
    K_cv.at<float>( 0, 0 )  = fx_;
    K_cv.at<float>( 1, 1 )  = fy_;
    K_cv.at<float>( 0, 2 )  = cx_;
    K_cv.at<float>( 1, 2 )  = cy_;
    K_cv.at<float>( 2, 2 )  = 1.0f;
    cv::undistort( distorted_image, dst, K_cv, dist_coef_ );
  }

  void Camera::unDistortImage( std::shared_ptr<Frame> frame, bool is_right )
  {
    cv::Mat K_cv           = cv::Mat::zeros( 3, 3, CV_32F );
    K_cv.at<float>( 0, 0 ) = fx_;
    K_cv.at<float>( 1, 1 ) = fy_;
    K_cv.at<float>( 0, 2 ) = cx_;
    K_cv.at<float>( 1, 2 ) = cy_;
    K_cv.at<float>( 2, 2 ) = 1.0f;
    if ( !is_right )
    {
      cv::undistort( frame->m_left_image, frame->m_left_image, K_cv, dist_coef_ );
    }
    else
    {
      cv::undistort( frame->right_image_, frame->right_image_, K_cv, dist_coef_ );
    }
  }

  double Camera::getFx() const
  {
    return fx_;
  }

  double Camera::getFy() const
  {
    return fy_;
  }

  double Camera::getCx() const
  {
    return cx_;
  }

  double Camera::getCy() const
  {
    return cy_;
  }

  double Camera::getBaseline() const
  {
    return baseline_;
  }

  Eigen::Matrix3d Camera::getK() const
  {
    return K_;
  }

  cv::Mat Camera::getDistCoef() const
  {
    return dist_coef_;
  }

  Sophus::SE3d Camera::getInvPose() const
  {
    return pose_inv_;
  }

  Sophus::SE3d Camera::getPose()
  {
    return m_pose;
  }
  //
}  // namespace lvio