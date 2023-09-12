#include <gtest/gtest.h>

#include "frontEnd.hpp"

TEST( FrontEndTest, TestGrabImageStereo )
{
  lvio::FrontEnd front_end;
  cv::Mat        left_image, right_image;
  double         time_stamp = 0.0;
  bool           result     = front_end.grabImageStereo( left_image, right_image, time_stamp );
  EXPECT_TRUE( result );
}

TEST( FrontEndTest, TestDetectFeatures )
{
  lvio::FrontEnd front_end;
  cv::Mat        left_image, right_image;
  double         time_stamp   = 0.0;
  bool           result       = front_end.grabImageStereo( left_image, right_image, time_stamp );
  int            num_features = front_end.detectFeatures();
  EXPECT_GT( num_features, 0 );
}

TEST( FrontEndTest, TestInitStereoFrame )
{
  lvio::FrontEnd front_end;
  bool           result = front_end.initStereoFrame();
  EXPECT_TRUE( result );
}

TEST( FrontEndTest, TestFindFeaturesInRight )
{
  lvio::FrontEnd front_end;
  int            num_matches = front_end.findFeaturesInRight();
  EXPECT_GT( num_matches, 0 );
}

TEST( FrontEndTest, TestBuildInitMap )
{
  lvio::FrontEnd front_end;
  bool           result = front_end.buildInitMap();
  EXPECT_TRUE( result );
}

TEST( FrontEndTest, TestInsertKeyFrame )
{
  lvio::FrontEnd front_end;
  bool           result = front_end.insertKeyFrame();
  EXPECT_TRUE( result );
}

TEST( FrontEndTest, TestTrack )
{
  lvio::FrontEnd front_end;
  bool           result = front_end.track();
  EXPECT_TRUE( result );
}

TEST( FrontEndTest, TestTriangulateNewPoints )
{
  lvio::FrontEnd front_end;
  int            num_points = front_end.triangulateNewMapPoints();
  EXPECT_GT( num_points, 0 );
}

TEST( FrontEndTest, TestEstimateCurrentPose )
{
  lvio::FrontEnd front_end;
  int            num_iterations = front_end.estimateCurrentPose();
  EXPECT_GT( num_iterations, 0 );
}

TEST( FrontEndTest, TestSetCamera )
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
  lvio::Camera      camera( fx, fy, cx, cy, baseline, pose, dist_coef );
  lvio::Camera::Ptr camera_ptr = std::make_shared<lvio::Camera>( camera );
  lvio::FrontEnd    front_end;

  front_end.setCamera( camera_ptr, camera_ptr );
  // EXPECT_EQ( front_end.left_camera_ptr_, camera_ptr );
  // EXPECT_EQ( front_end.right_camera_ptr_, camera_ptr );
  EXPECT_TRUE( front_end.left_camera_ptr_ == camera_ptr );
  EXPECT_TRUE( front_end.right_camera_ptr_ == camera_ptr );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}