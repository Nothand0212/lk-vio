#include <gtest/gtest.h>

#include "camera.hpp"

TEST( CameraTest, Basic )
{
  // 创建一个相机对象
  double       fx = 640, fy = 480, cx = 320, cy = 240, baseline = 500.0;
  Sophus::SE3d pose;
  cv::Mat      dist_coef = cv::Mat::zeros( 4, 1, CV_32F );
  // 对dist_coef赋值
  for ( int i = 0; i < dist_coef.rows; i++ )
  {
    dist_coef.at<float>( i, 0 ) = 0.1 * i;
  }
  lvio::Camera camera( fx, fy, cx, cy, baseline, pose, dist_coef );

  // 测试相机的内参是否正确
  EXPECT_EQ( camera.getFx(), fx );
  EXPECT_EQ( camera.getFy(), fy );
  EXPECT_EQ( camera.getCx(), cx );
  EXPECT_EQ( camera.getCy(), cy );


  // 测试相机的畸变系数是否正确
  for ( int i = 0; i < dist_coef.rows; i++ )
  {
    EXPECT_EQ( camera.getDistCoef().at<float>( i, 0 ), dist_coef.at<float>( i, 0 ) );
  }

  // 测试相机的像素坐标转换是否正确
  Eigen::Vector3d point_camera( 1.0, 2.0, 3.0 );
  Eigen::Vector2d point_pixel = camera.cameraToPixel( point_camera );
  EXPECT_NEAR( point_pixel( 0 ), 320.625, 1e-6 );
  EXPECT_NEAR( point_pixel( 1 ), 480.625, 1e-6 );

  // 测试相机的像素坐标反转换是否正确
  Eigen::Vector3d point_camera2 = camera.pixelToCamera( point_pixel, 3.0 );
  EXPECT_NEAR( point_camera2( 0 ), 1.171875, 1e-6 );
  EXPECT_NEAR( point_camera2( 1 ), 1.7578125, 1e-6 );
  EXPECT_NEAR( point_camera2( 2 ), 3.0, 1e-6 );
}

TEST( CameraTest, WorldToCamera )
{
  // 创建一个相机对象
  Eigen::Matrix3d K;
  K << 640, 0, 320, 0, 480, 240, 0, 0, 1;
  double       baseline = 500.0;
  Sophus::SE3d pose;
  cv::Mat      dist_coef = cv::Mat::zeros( 4, 1, CV_32F );
  // 对dist_coef赋值
  for ( int i = 0; i < dist_coef.rows; i++ )
  {
    dist_coef.at<float>( i, 0 ) = 0.1 * i;
  }
  lvio::Camera camera( K, baseline, pose, dist_coef );

  // 创建一个世界坐标系下的点
  Eigen::Vector3d point_world( 1.0, 2.0, 3.0 );

  // 创建一个相机到世界的变换矩阵
  Sophus::SE3d T_c_w = Sophus::SE3d::rotX( 0.1 ) * Sophus::SE3d::rotY( 0.2 ) * Sophus::SE3d::rotZ( 0.3 ) * Sophus::SE3d::trans( Eigen::Vector3d( 4.0, 5.0, 6.0 ) );

  // 计算相机坐标系下的点
  Eigen::Vector3d point_camera = camera.worldToCamera( point_world, T_c_w );

  // 计算期望值
  Eigen::Vector3d expected_point_camera = camera.getPose() * T_c_w * point_world;

  // 测试结果是否正确
  EXPECT_NEAR( point_camera( 0 ), expected_point_camera( 0 ), 1e-6 );
  EXPECT_NEAR( point_camera( 1 ), expected_point_camera( 1 ), 1e-6 );
  EXPECT_NEAR( point_camera( 2 ), expected_point_camera( 2 ), 1e-6 );
}

TEST( CameraTest, CameraToWorld )
{
  // 创建一个相机对象
  double       fx = 640, fy = 480, cx = 320, cy = 240, baseline = 500.0;
  Sophus::SE3d pose;
  cv::Mat      dist_coef = cv::Mat::zeros( 4, 1, CV_32F );
  // 对dist_coef赋值
  for ( int i = 0; i < dist_coef.rows; i++ )
  {
    dist_coef.at<float>( i, 0 ) = 0.1 * i;
  }
  lvio::Camera camera( fx, fy, cx, cy, baseline, pose, dist_coef );

  // 创建一个相机坐标系下的点
  Eigen::Vector3d point_camera( 1.0, 2.0, 3.0 );

  // 创建一个相机到世界的变换矩阵
  Sophus::SE3d T_c_w = Sophus::SE3d::rotX( 0.1 ) * Sophus::SE3d::rotY( 0.2 ) * Sophus::SE3d::rotZ( 0.3 ) * Sophus::SE3d::trans( Eigen::Vector3d( 4.0, 5.0, 6.0 ) );

  // 计算世界坐标系下的点
  Eigen::Vector3d point_world = camera.cameraToWorld( point_camera, T_c_w );

  // 计算期望值
  Eigen::Vector3d expected_point_world = T_c_w.inverse() * camera.getInvPose() * point_camera;

  // 测试结果是否正确
  EXPECT_NEAR( point_world( 0 ), expected_point_world( 0 ), 1e-6 );
  EXPECT_NEAR( point_world( 1 ), expected_point_world( 1 ), 1e-6 );
  EXPECT_NEAR( point_world( 2 ), expected_point_world( 2 ), 1e-6 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}