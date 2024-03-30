#pragma once

namespace lk_vio
{
  struct CameraIntrinsics
  {
    double fx{ 0.0f };
    double fy{ 0.0f };
    double cx{ 0.0f };
    double cy{ 0.0f };
    double k1{ 0.0f };
    double k2{ 0.0f };
    double p1{ 0.0f };
    double p2{ 0.0f };
    double k3{ 0.0f };
  };

  struct CameraParams
  {
    CameraIntrinsics left;
    CameraIntrinsics right;

    float baseline;
    int   width;
    int   height;
    bool  need_undistortion;
  };


  struct MapParams
  {
    int active_map_size;
    int init_landmark_size;
  };

  struct TrackingStatusParams
  {
    int init_good;
    int track_good;
    int track_bad;
  };

  struct ExtractorParams
  {
    int   init_features_num;
    int   track_features_num;
    float scale_factor;
    int   pyramid_level;
    int   init_fast_threshold;
    int   min_fast_threshold;
  };

  struct BackEndParams
  {
    bool activate;
  };

  struct LoopClosureParams
  {
    bool  activate;
    bool  show_result;
    float max_threshold;
    float min_threshold;
    int   min_database_size;
    int   pyramid_level;
  };

  struct ViewerParams
  {
    bool show_extractor_result_cv;
    bool show_lk_matcher_result_cv;
  };

  struct IMUParams
  {
    bool activate;
  };

}  // namespace lk_vio
