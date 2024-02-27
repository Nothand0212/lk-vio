#ifndef LVIO_CAMERA_HPP
#define LVIO_CAMERA_HPP

#include <memory>

#include "Eigen/Core"
#include "frame.hpp"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"
namespace lvio
{
  class Camera
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;

    Camera() = default;

    // 使用成员初始化列表的方式可以更加高效，因为它可以在构造函数开始时就初始化所有成员变量，而不需要等到构造函数体中进行赋值操作。
    // 这样可以避免不必要的构造函数调用和内存分配，从而提高程序的性能。此外，使用成员初始化列表的方式还可以确保所有成员变量都被正确地初始化，从而避免了一些潜在的错误。
    Camera( double fx, double fy, double cx, double cy, double baseline,
            const Sophus::SE3d &pose, const cv::Mat dist_coef )
        : fx_( fx ), fy_( fy ), cx_( cx ), cy_( cy ), baseline_( baseline ), m_pose( pose ), dist_coef_( dist_coef )
    {
      K_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
      pose_inv_ = m_pose.inverse();
    }

    Camera( const Eigen::Matrix3d &K, const double &baseline,
            const Sophus::SE3d &pose, const cv::Mat &dist_coef )
        : K_( K ), baseline_( baseline ), m_pose( pose ), dist_coef_( dist_coef )
    {
      fx_ = K_( 0, 0 );
      fy_ = K_( 1, 1 );
      cx_ = K_( 0, 2 );
      cy_ = K_( 1, 2 );

      pose_inv_ = m_pose.inverse();
    }

    void         setPose( const Sophus::SE3d &pose );
    Sophus::SE3d getPose();

    Eigen::Vector3d worldToCamera( const Eigen::Vector3d &point_world,
                                   const Sophus::SE3d &   T_c_w );
    Eigen::Vector3d cameraToWorld( const Eigen::Vector3d &point_camera,
                                   const Sophus::SE3d &   T_c_w );

    Eigen::Vector2d cameraToPixel( const Eigen::Vector3d &point_camera );
    Eigen::Vector3d pixelToCamera( const Eigen::Vector2d &point_pixel,
                                   const double &         depth = 1 );

    Eigen::Vector3d pixelToWorld( const Eigen::Vector2d &point_pixel,
                                  const Sophus::SE3d &   T_c_w,
                                  const double &         depth = 1 );
    Eigen::Vector2d worldToPixel( const Eigen::Vector3d &point_world,
                                  const Sophus::SE3d &   T_c_w );

    void unDistortImage( cv::Mat &src, cv::Mat &dst );
    void unDistortImage( std::shared_ptr<Frame> frame, bool is_right = false );

    double          getFx() const;
    double          getFy() const;
    double          getCx() const;
    double          getCy() const;
    double          getBaseline() const;
    Eigen::Matrix3d getK() const;
    cv::Mat         getDistCoef() const;
    Sophus::SE3d    getInvPose() const;

  private:
    double          fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    double          baseline_ = 0;
    Sophus::SE3d    m_pose;
    Sophus::SE3d    pose_inv_;
    Eigen::Matrix3d K_;
    cv::Mat         dist_coef_;
  };
  //
}  // namespace lvio

#endif  // LVIO_CAMERA_HPP