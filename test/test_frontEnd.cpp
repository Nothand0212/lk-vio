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
  int            num_points = front_end.triangulateNewPoints();
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
  lvio::FrontEnd    front_end;
  lvio::Camera::Ptr left_camera  = std::make_shared<lvio::Camera>();
  lvio::Camera::Ptr right_camera = std::make_shared<lvio::Camera>();
  front_end.setCamera( left_camera, right_camera );
  EXPECT_EQ( front_end.left_camera_, left_camera );
  EXPECT_EQ( front_end.right_camera_, right_camera );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}